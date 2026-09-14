# laptop_ai/media_server.py
import os
import json
import asyncio
from aiohttp import web
import glob
import datetime

MEDIA_DIR = "media"

class MediaServer:
    def __init__(self, host="0.0.0.0", port=8080):
        self.host = host
        self.port = port
        self.app = web.Application()
        self.runner = None
        self.site = None

        # add_static() resolves the path strictly, so the dir MUST exist before this line.
        # (The makedirs in start() runs too late — __init__ would already have crashed.)
        os.makedirs(MEDIA_DIR, exist_ok=True)

        # Routes
        self.app.router.add_get('/media', self.list_media)
        self.app.router.add_static('/media', path=MEDIA_DIR, show_index=True)
        # self.app.router.add_delete('/media/{filename}', self.delete_media)

    async def start(self):
        """Start the async HTTP server"""
        if not os.path.exists(MEDIA_DIR):
            os.makedirs(MEDIA_DIR)
            
        self.runner = web.AppRunner(self.app)
        await self.runner.setup()
        self.site = web.TCPSite(self.runner, self.host, self.port)
        await self.site.start()
        print(f"📂 MEDIA SERVER STARTED: http://{self.host}:{self.port}/media")

    async def stop(self):
        if self.runner:
            await self.runner.cleanup()

    async def list_media(self, request):
        """Return JSON list of media files with metadata"""
        files = []
        # List jpg and mp4
        for ext in ['*.jpg', '*.mp4']:
            for filepath in glob.glob(os.path.join(MEDIA_DIR, ext)):
                try:
                    stats = os.stat(filepath)
                    filename = os.path.basename(filepath)
                    files.append({
                        "name": filename,
                        "url": f"/media/{filename}",
                        "size": stats.st_size,
                        "date": datetime.datetime.fromtimestamp(stats.st_mtime).isoformat(),
                        "type": "video" if filename.endswith(".mp4") else "photo"
                    })
                except Exception as e:
                    print(f"Error scanning {filepath}: {e}")
        
        # Sort by date new -> old
        files.sort(key=lambda x: x["date"], reverse=True)
        
        return web.json_response({"files": files})
