import asyncio
import threading
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from fastapi import FastAPI, Request
from fastapi.staticfiles import StaticFiles
from fastapi.responses import JSONResponse, FileResponse

from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaPlayer, MediaRelay
import pkg_resources

from ament_index_python.packages import get_package_share_directory
from pathlib import Path


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
        # self.declare_parameter('stun_urls', ['stun:stun.l.google.com:19302'])

        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = int(self.get_parameter('fps').value)
        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = int(self.get_parameter('port').value)
        # self.stun_urls = [s for s in self.get_parameter('stun_urls').get_parameter_value().string_array_value]
        
        self.declare_parameter('stun_urls', '')  # default as CSV string
        raw = self.get_parameter('stun_urls').value
        if isinstance(raw, (list, tuple)):
            self.stun_urls = [s for s in raw if isinstance(s, str) and s.strip()]
        elif isinstance(raw, str):
            self.stun_urls = [s.strip() for s in raw.split(',') if s.strip()]
        else:
            self.stun_urls = []

        # FastAPI app
        self.app = FastAPI(title='ROS Streamer')

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

            if self._player and self._player.video:
                # Fan-out the same camera stream to this peer
                pc.addTrack(self._relay.subscribe(self._player.video))
            else:
                raise RuntimeError("Camera player not initialized or has no video track.")

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
