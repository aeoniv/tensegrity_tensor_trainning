import asyncio
import websockets
import json
import random

async def test_control():
    uri = "ws://127.0.0.1:8766"
    print(f"Connecting to {uri}...")
    try:
        async with websockets.connect(uri) as websocket:
            print("Connected!")
            
            # Test all 12 indices
            for i in range(12):
                print(f"Testing Index {i}...")
                
                # Extend
                msg = {"type": "piston_move", "index": i, "value": 1.0}
                await websocket.send(json.dumps(msg))
                print(f"  Sent: {msg}")
                await asyncio.sleep(0.5)
                
                # Retract
                msg = {"type": "piston_move", "index": i, "value": 0.0}
                await websocket.send(json.dumps(msg))
                print(f"  Sent: {msg}")
                await asyncio.sleep(0.5)
                
            print("Test Complete.")
    except Exception as e:
        print(f"Connection Failed: {e}")

if __name__ == "__main__":
    asyncio.run(test_control())
