import asyncio
import websockets
import json

async def test():
    async with websockets.connect('ws://localhost:5001/motor') as websocket:
        await websocket.send(json.dumps({"type": "voice", "command": "red"}))
        response = await websocket.recv()
        print(f"Received: {response}")
        
        # Now check the nav status via API
        import urllib.request
        resp = urllib.request.urlopen('http://localhost:5001/api/navigation/status')
        print(resp.read().decode())

asyncio.run(test())
