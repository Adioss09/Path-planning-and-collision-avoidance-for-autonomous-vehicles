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
                # Build mock UnityStateMessage matching the strict Pydantic nested Vector3 schema
                state = {
                    "timestamp": time.time(),
                    "scenario": {
                        "id": "cattle_crossing",
                        "time": time.time()
                    },
                    "vehicle": {
                        "id": "ego",
                        "position": {
                            "x": 120.0,
                            "y": 0.0,
                            "z": 500.0 - (i * 20)
                        },
                        "heading": -1.57,
                        "speed": 35.0
                    },
                    "objects": [
                        {
                            "id": "CATTLE_01",
                            "type": "animal",
                            "position": {
                                "x": 100.0,
                                "y": 0.0,
                                "z": 400.0
                            },
                            "velocity": {
                                "x": 5.0,
                                "z": 0.0
                            }
                        }
                    ]
                }
                
                print(f"[UNITY] Sending State {i+1}...")
                await websocket.send(json.dumps(state))
                
                response = await websocket.recv()
                data = json.loads(response)
                
                print(f"[BRAIN] Received Decision: Risk={data.get('decision', {}).get('risk')}, Action={data.get('decision', {}).get('action')}, TargetSpeed={data.get('decision', {}).get('target_speed'):.1f}")
                print(f"[BRAIN] Trajectory Length: {len(data.get('trajectory', []))} points\n")
                
                await asyncio.sleep(0.5)
                
    except ConnectionRefusedError:
        print("Error: Could not connect to server. Ensure unity_server.py is running.")
        
if __name__ == "__main__":
    asyncio.run(mock_unity_client())
