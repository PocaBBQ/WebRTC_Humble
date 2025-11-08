import asyncio
import threading
import rclpy
import pkg_resources
import av, time
import subprocess, re, json
from rclpy.node import Node
from rclpy.parameter import Parameter
from pathlib import Path
from fastapi import FastAPI, Request
from fastapi.staticfiles import StaticFiles
from fastapi.responses import JSONResponse, FileResponse
from aiortc import RTCPeerConnection, RTCSessionDescription, MediaStreamTrack
from aiortc.contrib.media import MediaPlayer, MediaRelay
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

class ResizedTrack(MediaStreamTrack):
    kind = "video"

    def __init__(self, source_track: MediaStreamTrack, width: int, height: int, fps: int | None):
        super().__init__()  # base class init
        self._source = source_track
        self._w = max(160, int(width))
        self._h = max(120, int(height))
        self._fps = int(fps) if fps else None
        self._min_interval = (1.0 / self._fps) if self._fps and self._fps > 0 else 0.0
        self._next_ts = 0.0

    async def recv(self) -> av.VideoFrame:
        while True:
            frame: av.VideoFrame = await self._source.recv()

            # Optional FPS limit: drop frames to meet target fps
            if self._min_interval > 0.0:
                now = time.time()
                if now < self._next_ts:
                    # skip this frame, grab next
                    continue
                self._next_ts = now + self._min_interval

            # Resize using libswscale via PyAV
            if frame.width != self._w or frame.height != self._h:
                frame = frame.reformat(width=self._w, height=self._h)

            return frame

class RosStreamerNode(Node):
    def __init__(self):
        super().__init__('ros_streamer_server')

        # Parameters
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8000)
        self.declare_parameter('stun_urls', '')  # default as CSV string
        # self.declare_parameter('stun_urls', ['stun:stun.l.google.com:19302'])        

        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = int(self.get_parameter('fps').value)
        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = int(self.get_parameter('port').value)
        # self.stun_urls = [s for s in self.get_parameter('stun_urls').get_parameter_value().string_array_value]

        # Probe camera capabilities (best-effort)
        self._caps = []
        try:
            out = subprocess.check_output(
                ["v4l2-ctl", "--list-formats-ext", "-d", self.device],
                stderr=subprocess.STDOUT, text=True
            )
            self._caps = self._parse_v4l2_caps(out)
            self.get_logger().info(f"Found {len(self._caps)} camera modes on {self.device}")
            if not self._caps:
                self.get_logger().warn("v4l2-ctl parsed 0 modes; enabling fallback modes.")
        except Exception as e:
            self.get_logger().warn(f"Could not probe caps for {self.device}: {e}\nEnabling fallback modes.")

        ## Optional plan if cannot get caps ##
        # if not self._caps:
        #     # Common C920/C920e MJPG modes — adjust if your camera differs
        #     fallback_res = [
        #         (1920,1080,[30]),
        #         (1280,720,[30,60]),
        #         (960,540,[30]),
        #         (848,480,[30]),
        #         (800,600,[30]),
        #         (640,480,[30,60]),
        #         (424,240,[30]),
        #         (320,240,[30])
        #     ]
        #     self._caps = [{"pixfmt":"MJPG","width":w,"height":h,"fps":fps} for (w,h,fps) in fallback_res]

        raw = self.get_parameter('stun_urls').value
        if isinstance(raw, (list, tuple)):
            self.stun_urls = [s for s in raw if isinstance(s, str) and s.strip()]
        elif isinstance(raw, str):
            self.stun_urls = [s.strip() for s in raw.split(',') if s.strip()]
        else:
            self.stun_urls = []

        # FastAPI app
        self.app = FastAPI(title='ROS Streamer')
        
        @self.app.get('/caps')
        async def caps():
            # Group by resolution for convenience: { "1280x720": [15,30], ... }
            by_res = {}
            for m in self._caps:
                key = f'{m["width"]}x{m["height"]}'
                by_res.setdefault(key, set())
                for f in m["fps"]:
                    by_res[key].add(f)
            # convert sets to sorted lists
            by_res = {k: sorted(list(v)) for k, v in by_res.items()}
            return JSONResponse({
                "device": self.device,
                "resolutions": sorted(by_res.keys(), key=lambda s: (int(s.split("x")[0])*int(s.split("x")[1]), s)),
                "fps_by_resolution": by_res
            })

        candidates = []
        # A) Installed location (share/<pkg>/static)
        try:
            share_dir = Path(get_package_share_directory('ros_streamer'))
            candidates.append(share_dir / 'static')
        except Exception:
            pass

        static_dir = next((p for p in candidates if p.exists()), None)
        if static_dir is None:
            raise RuntimeError(
                "Static directory not found. Expected one of:\n" +
                "\n".join(str(p) for p in candidates) +
                "\nMake sure 'ros_streamer/static/index.html' exists and rebuild with `colcon build --symlink-install`."
            )

        self.app.mount('/static', StaticFiles(directory=str(static_dir)), name='static')

        @self.app.get('/')
        async def index():
            return FileResponse(str(static_dir / 'index.html'))

        self.pcs = set()

        # ---- Single shared camera + relay ----
        self._relay = MediaRelay()

        def _make_player():
            opts = {
                'video_size': f'{self.width}x{self.height}',
                'framerate': f'{self.fps}'
            }
            return MediaPlayer(self.device, format='v4l2', options=opts)

        # Create the camera once at startup
        self._player = _make_player()

        @self.app.post('/offer')
        async def offer(request: Request):
            params = await request.json()
            offer = RTCSessionDescription(sdp=params['sdp'], type=params['type'])

            # Optional constraints from client:
            cons = params.get('constraints', {}) or {}
            try:
                cw = int(cons.get('width',  self.width))
                ch = int(cons.get('height', self.height))
                cfps = int(cons.get('fps', 0)) or None
            except Exception:
                cw, ch, cfps = self.width, self.height, None

            cfg = {} if not self.stun_urls else {"iceServers": [{"urls": self.stun_urls}]}
            pc = RTCPeerConnection(configuration=cfg)
            self.pcs.add(pc)

            @pc.on('connectionstatechange')
            async def on_state_change():
                state = pc.connectionState
                self.get_logger().info(f'Peer state: {state}')
                if state in ('failed', 'closed', 'disconnected'):
                    await pc.close()
                    self.pcs.discard(pc)

            if not (self._player and self._player.video):
                raise RuntimeError("Camera not initialized")

            base_track = self._relay.subscribe(self._player.video)
            # wrap per-peer transform
            resized = ResizedTrack(base_track, width=cw, height=ch, fps=cfps)
            pc.addTrack(resized)

            await pc.setRemoteDescription(offer)
            answer = await pc.createAnswer()
            await pc.setLocalDescription(answer)
            return JSONResponse({'sdp': pc.localDescription.sdp, 'type': pc.localDescription.type})

        import uvicorn
        self._uvicorn_config = uvicorn.Config(self.app, host=self.host, port=self.port, log_level='info')
        self._uvicorn_server = uvicorn.Server(config=self._uvicorn_config)
        self._server_thread = threading.Thread(target=self._run_server, daemon=True)
        self._server_thread.start()
        self.get_logger().info(f'ROS Streamer started at http://{self.host}:{self.port}/')

    def _run_server(self):
        asyncio.set_event_loop(asyncio.new_event_loop())
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self._uvicorn_server.serve())

    def destroy_node(self):
        async def _close():
            await asyncio.gather(*[pc.close() for pc in list(self.pcs)], return_exceptions=True)
            self.pcs.clear()

        try:
            asyncio.run(_close())
        except RuntimeError:
            pass
        return super().destroy_node()
    
    def _parse_v4l2_caps(self, text: str):
        """
        Robustly parse `v4l2-ctl --list-formats-ext` output.

        Returns a list of modes: [{pixfmt, width, height, fps:[...]}]
        Accepts lines like:
        [0]: 'MJPG' (Motion-JPEG)
            Size: Discrete 1280x720
            Interval: Discrete 1/30s (30.000 fps)
        or:
        [1]: 'YUYV' (YUYV 4:2:2)
            Size: Discrete 640x480
            Interval: Discrete 333333/10000000s (29.970 fps)
        or the "Frame size" / "Frame interval" wording seen on some kernels.
        """
        modes = []
        fmt = None
        w = h = None

        # Normalize lines (strip & collapse spaces)
        lines = [ln.strip() for ln in text.splitlines()]

        # Regexes for different formats of the same info
        re_fmt    = re.compile(r"Pixel\s*Format:\s*'([^']+)'|^\[\d+\]\s*:\s*'([^']+)'")
        re_size1  = re.compile(r"(?:Size|Frame size)\s*:\s*Discrete\s*(\d+)\s*x\s*(\d+)", re.I)
        re_size2  = re.compile(r"^\s*(\d+)\s*x\s*(\d+)\s*$")  # rare variant
        re_intv1  = re.compile(r"(?:Interval|Frame interval)\s*:\s*Discrete\s*([\d\.]+)\s*fps", re.I)
        re_intv2  = re.compile(r"\(([\d\.]+)\s*fps\)", re.I)

        def _add_mode(ff, width, height, fps):
            existing = next((m for m in modes if m["pixfmt"]==ff and m["width"]==width and m["height"]==height), None)
            if not existing:
                existing = {"pixfmt": ff, "width": width, "height": height, "fps": []}
                modes.append(existing)
            if fps not in existing["fps"]:
                existing["fps"].append(fps)

        for ln in lines:
            m = re_fmt.search(ln)
            if m:
                fmt = m.group(1) or m.group(2)
                w = h = None
                continue

            m = re_size1.search(ln) or re_size2.search(ln)
            if m:
                w, h = int(m.group(1)), int(m.group(2))
                continue

            if fmt and w and h:
                m = re_intv1.search(ln) or re_intv2.search(ln)
                if m:
                    try:
                        fps_val = int(round(float(m.group(1))))
                        if fps_val > 0:
                            _add_mode(fmt, w, h, fps_val)
                    except Exception:
                        pass

        # Sort for nice UI
        for m in modes:
            m["fps"].sort()
        modes.sort(key=lambda k: (k["width"]*k["height"], k["width"], k["height"], k["pixfmt"]))
        return modes


def main(args=None):
    rclpy.init(args=args)
    node = RosStreamerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down ROS Streamer...')
        node.destroy_node()
        rclpy.shutdown()
