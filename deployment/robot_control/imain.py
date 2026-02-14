import mujoco
import mujoco.viewer
import time
import os
import sys
import asyncio
import websockets
import json
import numpy as np

# Path to the model
# 1. Docker Path
DOCKER_PATH = "/app/mujoco/menagerie/unitree_g1/scene.xml"
# 2. Local Relative Path (assuming script is in deployment/robot_control)
LOCAL_PATH = os.path.join(os.path.dirname(__file__), "../../public/mujoco/menagerie/unitree_g1/scene.xml")

MODEL_PATH = DOCKER_PATH if os.path.exists(DOCKER_PATH) else os.path.abspath(LOCAL_PATH)

# Global set of connected clients
connected_clients = set()

async def broadcast_state(model, data):
    if not connected_clients:
        return
        
    # Serialize state
    # Extract Tensegrity Node Positions
    tensegrity_state = {}
    # We can optimize this by caching indices, but loop over nbody (small) is fine for now
    for i in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        if name and name.startswith("node_"):
            tensegrity_state[name] = data.xpos[i].tolist()

    message = json.dumps({
        "time": data.time,
        "qpos": data.qpos.tolist(),
        "tensegrity": tensegrity_state
    })
    
    # Broadcast to all
    # Websockets handles the loop
    await asyncio.gather(*[client.send(message) for client in connected_clients])

async def handler(websocket):
    print("Client connected!")
    connected_clients.add(websocket)
    try:
        await websocket.wait_closed()
    finally:
        connected_clients.remove(websocket)
        print("Client disconnected.")

async def run_simulation(model, data):
    print("Starting simulation loop with WebSocket server...")
    
    # Check if we can launch viewer (local only)
    viewer = None
    try:
        viewer = mujoco.viewer.launch_passive(model, data)
        # Enable Label ONLY for 'Selection' (The clicked object)
        viewer.opt.label = mujoco.mjtLabel.mjLABEL_SELECTION
        print("Native Viewer Launched.")
    except Exception:
        print("Running Headless (Viewer launch failed).")

    # Simulation Params
    dt = model.opt.timestep
    if dt == 0: dt = 0.002
    
    steps = 0
    
    while True:
        frame_start = time.time()
        
        # Physics Step
        if viewer is not None:
            with viewer.lock():
                mujoco.mj_step(model, data)
            viewer.sync()
        else:
            mujoco.mj_step(model, data)
        
        steps += 1
        
        # Broadcast State (Async)
        # We broadcast every step or throttle? 
        # 500Hz might be too fast for WS if network is slow. 
        # Let's throttle to ~60Hz broadcast (every 8 steps roughly if dt=0.002)
        if steps % 8 == 0:
            await broadcast_state(model, data)
            
        # Log occasionally
        if steps % 500 == 0:
            print(f"SimTime: {data.time:.2f}s | Connections: {len(connected_clients)}")

        # Timing (simple sleep)
        elapsed = time.time() - frame_start
        if elapsed < dt:
            await asyncio.sleep(dt - elapsed) # Async sleep to yield to WS

async def main_async():
    print("Initializing MuJoCo Native Simulation - Unitree G1...")
    
    print(f"Current Working Directory: {os.getcwd()}")
    resolved_path = os.path.abspath(MODEL_PATH)
    print(f"Resolved Model Path: {resolved_path}")

    if not os.path.exists(resolved_path):
        print(f"Error: Model not found at {resolved_path}")
        # List dir to help debug
        target_dir = os.path.dirname(resolved_path)
        if os.path.exists(target_dir):
            print(f"Contents of {target_dir}:")
            print(os.listdir(target_dir))
        else:
            print(f"Directory {target_dir} does not exist.")
        return

    try:
        print(f"Loading model from {resolved_path}")
        model = mujoco.MjModel.from_xml_path(resolved_path)
        data = mujoco.MjData(model)
        
        print("Starting WebSocket Server on port 8765...")
        
        # Debug: Print Actuators
        print(f"Model has {model.nu} actuators.")
        for i in range(model.nu):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            print(f" - Actuator {i}: {name}")

        async with websockets.serve(handler, "0.0.0.0", 8765):
            await run_simulation(model, data)

    except asyncio.CancelledError:
        print("Simulation Cancelled.")
    except Exception as e:
        print(f"Critical Error: {e}")
        import traceback
        traceback.print_exc()

def main():
    try:
        asyncio.run(main_async())
    except KeyboardInterrupt:
        print("Simulation Stopped by User.")

if __name__ == "__main__":
    main()
