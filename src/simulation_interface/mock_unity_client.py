import asyncio
import websockets
import json
import time

async def mock_unity_client():
    uri = "ws://localhost:8765"
    print(f"Connecting to Python Brain at {uri}...")
    
    try:
        async with websockets.connect(uri) as websocket:
            print("Connected! Simulating Unity environment...\n")
            
            for i in range(5):
                # Build mock UnityStateMessage
                state = {
                    "timestamp": time.time(),
                    "scenario_id": "cattle",
                    "vehicle": {
                        "x": 120.0,
                        "y": 500.0 - (i * 20),
                        "heading": -1.57,
                        "speed": 35.0
                    },
                    "objects": [
                        {
                            "id": "CATTLE_01",
                            "class": "animal",
                            "x": 100.0,
                            "y": 400.0,
                            "vx": 5.0,
                            "vy": 0.0
                        }
                    ]
                }
                
                print(f"[UNITY] Sending State {i+1}...")
                await websocket.send(json.dumps(state))
                
                response = await websocket.recv()
                data = json.loads(response)
                
                print(f"[BRAIN] Received Decision: Risk={data.get('risk')}, Action={data.get('action')}, TargetSpeed={data.get('target_speed'):.1f}")
                print(f"[BRAIN] Trajectory Length: {len(data.get('trajectory', []))} points\n")
                
                await asyncio.sleep(0.5)
                
    except ConnectionRefusedError:
        print("Error: Could not connect to server. Ensure unity_server.py is running.")
        
if __name__ == "__main__":
    asyncio.run(mock_unity_client())
