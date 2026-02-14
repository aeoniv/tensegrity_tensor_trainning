import mujoco
import mujoco.viewer
import time
import os
import sys
import asyncio
import websockets
import json
import numpy as np
import math

# Path to the model (Puppet Scene)
MODEL_PATH = "public/mujoco/menagerie/unitree_g1/scene_puppet.xml"

# Global set of connected clients
connected_clients = set()

async def broadcast_state(data):
    if not connected_clients:
        return
        
    # Serialize state
    message = json.dumps({
        "time": data.time,
        "qpos": data.qpos.tolist(),
        # "qvel": data.qvel.tolist() 
    })
    
    # Broadcast to all
    await asyncio.gather(*[client.send(message) for client in connected_clients])

# Global Control State
target_controls = []

async def handler(websocket):
    print("Client connected!")
    connected_clients.add(websocket)
    try:
        async for message in websocket:
            try:
                msg = json.loads(message)
                if msg.get("type") == "piston_move":
                    idx = msg.get("index")
                    val = msg.get("value")
                    # Validate
                    if idx is not None and val is not None:
                        if 0 <= idx < len(target_controls):
                            # Clamp value 0.0 to 1.0
                            val = max(0.0, min(1.0, float(val)))
                            target_controls[idx] = val
                            # print(f"Set Piston {idx} to {val}") # Debug (spammy)
            except json.JSONDecodeError:
                pass
            except Exception as e:
                print(f"Error handling message: {e}")
    except websockets.exceptions.ConnectionClosed:
        pass
    except Exception as e:
        print(f"WebSocket Error: {e}")
    finally:
        connected_clients.remove(websocket)
        print("Client disconnected.")

async def run_simulation(model, data):
    print("Starting Puppet Simulation loop with WebSocket server...")
    print("Controls:")
    print("  [SPACE]: Pause/Resume")
    print("  [O]: Breath Octahedron (Inner)")
    print("  [C]: Breath Cube (Outer)")
    print("  [P]: Breath Connecting Pistons (Rhombic Edges)")
    
    # Check if we can launch viewer (local only)
    viewer = None
    try:
        viewer = mujoco.viewer.launch_passive(model, data)
        viewer.opt.label = mujoco.mjtLabel.mjLABEL_SELECTION
        print("Native Viewer Launched.")
    except Exception:
        print("Running Headless (Viewer launch failed).")

    # Simulation Params
    dt = model.opt.timestep
    if dt == 0: dt = 0.002
    
    # Simulation Params
    dt = model.opt.timestep
    if dt == 0: dt = 0.002
    
    steps = 0
    
    # MAPPING CONFIGURATION
    # 24 Main Joints
    joint_names = [
        "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", 
        "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", 
        "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
        "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
        "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint",
        "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
        "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint"
    ]
    
    # Store indices for fast access
    joint_ids = []
    piston_act_ids = []
    
    for name in joint_names:
        # Robot Joint ID (qpos address)
        # qpos is not 1:1 with joint_id if freejoint exists. 
        # Better to get qpos address.
        j_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        qpos_adr = model.jnt_qposadr[j_id]
        joint_ids.append(qpos_adr)
        
        # Piston Actuator ID
        # Piston name format: "act_{name}_piston_a"
        p_act_name = f"act_{name}_piston_a"
        p_act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, p_act_name)
        piston_act_ids.append(p_act_id)

    print(f"Mapped {len(joint_ids)} joints to pistons.")
    
    # Robot Control Indices (We need to actuate the robot joints)
    # The 'joint_ids' are qpos addresses (sensing).
    # We need actuator indices to drive them (control).
    # We kept G1 actuators, so they should exist.
    robot_act_ids = []
    for name in joint_names:
        # G1 Actuator names usually match joint names in the Menagerie XML, 
        # but let's check g1.xml or just try the joint name.
        # In generate_puppet we kept the <actuator> block.
        # It has <position name="..." joint="...">
        # Usually the name is the joint name.
        r_act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        robot_act_ids.append(r_act_id)
        
    print(f"Found {len(robot_act_ids)} robot actuators.")

    # --- MAPPING CONFIGURATION ---
    # 12 Arm Joints (Cube Pistons)
    arm_joint_names = [
        "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
        "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint",
        "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
        "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint"
    ]
    
    # Store indices for fast access
    # Maps user index [0-11] -> (Piston Actuator ID, Robot Joint Actuator ID)
    control_map = []
    
    print(f"Mapping {len(arm_joint_names)} Arm Joints (Cube) for Manual Control...")
    
    for name in arm_joint_names:
        # 1. Piston Actuator
        # Name format from generate_puppet.py: "act_piston_{joint_name}_piston_a"
        p_act_name = f"act_piston_{name}_piston_a"
        p_act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, p_act_name)
        
        # 2. Robot Joint Actuator
        # Name format: "{joint_name}" (standard G1 actuator names)
        r_act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        
        if p_act_id != -1 and r_act_id != -1:
            control_map.append({
                "name": name,
                "piston_id": p_act_id,
                "robot_id": r_act_id,
                "target_val": 0.0 # Default state
            })
            print(f"  [Mapped] {name}: Piston {p_act_id} <-> Robot {r_act_id}")
        else:
            print(f"  [ERROR] Could not find actuators for {name} (P:{p_act_id}, R:{r_act_id})")

    # Global State for Controls (referenced by websocket handler)
    global target_controls
    target_controls = [0.0] * len(control_map) # 0.0 to 1.0 range usually, or mapped to limits

    # Helper: Map 0-1 input to Actuator Range
    def map_range(val, min_out, max_out):
        # Input assumed 0.0 to 1.0? Or -1 to 1?
        # Let's assume input is normalized 0.0 (retracted) to 1.0 (extended)
        # Verify generate_puppet.py limits:
        # limit_comp (approx -0.6) to limit_ext (approx 0.25)
        return min_out + val * (max_out - min_out)
    
    # Let's get the range from the first mapped piston to ensure consistency
    p_range_min = -0.1 # Fallback
    p_range_max = 0.1
    if control_map:
        pid = control_map[0]['piston_id']
        p_range_min = model.actuator_ctrlrange[pid][0]
        p_range_max = model.actuator_ctrlrange[pid][1]
        print(f"Detected Piston Control Range: [{p_range_min:.3f}, {p_range_max:.3f}]")

    # Scaling for Robot Joint (Robot needs to move MORE than piston usually, or less?)
    # Piston: meters. Robot: radians.
    # 10.0 scale from previous code.
    robot_scale = 10.0 

    while True:
        frame_start = time.time()
        
        # Process incoming messages (Handled by async handler modifying `target_controls`)
        # Here we just apply `target_controls` to the physics
        
        for i, mapping in enumerate(control_map):
            if i >= len(target_controls): break
            
            # User Input (0.0 to 1.0)
            u_val = target_controls[i]
            
            # Map to Piston Physics Range
            p_val = map_range(u_val, p_range_min, p_range_max)
            
            # Map to Robot Joint Physics Range (simple scaling for now)
            # Center of piston range might not be 0 robot angle.
            # Let's assume Middle Piston = 0 Robot.
            # Midpoint = (min + max) / 2
            p_mid = (p_range_min + p_range_max) / 2
            r_val = (p_val - p_mid) * robot_scale
            
            # Apply to Piston Actuator
            data.ctrl[mapping['piston_id']] = p_val
            
            # Apply to Robot Actuator
            data.ctrl[mapping['robot_id']] = r_val

        # Physics Step
        if viewer is not None:
            with viewer.lock():
                mujoco.mj_step(model, data)
            viewer.sync()
        else:
            mujoco.mj_step(model, data)
        
        steps += 1
        
        # Broadcast State (Async)
        if steps % 8 == 0:
            await broadcast_state(data)
            
        # Log occasionally
        if steps % 500 == 0:
            # Print status of first active control
            if control_map:
                c0 = control_map[0]
                print(f"Time:{data.time:.1f}s | {c0['name']}: In({target_controls[0]:.2f}) -> P({data.ctrl[c0['piston_id']]:.2f}) / R({data.ctrl[c0['robot_id']]:.2f})")

        elapsed = time.time() - frame_start
        if elapsed < dt:
            await asyncio.sleep(dt - elapsed)
        else:
            # Force yield to allow WebSocket processing even if lagging
            await asyncio.sleep(0) 

async def handler(websocket):
    print("Client connected!")
    connected_clients.add(websocket)
    try:
        async for message in websocket:
            try:
                msg = json.loads(message)
                if msg.get("type") == "piston_move":
                    idx = msg.get("index")
                    val = msg.get("value")
                    # Validate
                    if idx is not None and val is not None:
                        if 0 <= idx < len(target_controls):
                            # Clamp value 0.0 to 1.0
                            val = max(0.0, min(1.0, float(val)))
                            target_controls[idx] = val
                            # print(f"Set Piston {idx} to {val}") # Debug (spammy)
            except json.JSONDecodeError:
                pass
            except Exception as e:
                print(f"Error handling message: {e}")
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        connected_clients.remove(websocket)
        print("Client disconnected.") 

async def main_async():
    print("Initializing MuJoCo Puppet Simulation...")
    
    print(f"Current Working Directory: {os.getcwd()}")
    resolved_path = os.path.abspath(MODEL_PATH)
    print(f"Resolved Model Path: {resolved_path}")

    if not os.path.exists(resolved_path):
        print(f"Error: Model not found at {resolved_path}")
        return

    try:
        print(f"Loading model from {resolved_path}")
        model = mujoco.MjModel.from_xml_path(resolved_path)
        data = mujoco.MjData(model)
        
        print("Starting WebSocket Server on port 8766...")
        
        # Debug: Print Actuators
        print(f"Model has {model.nu} actuators.")
        for i in range(model.nu):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            print(f" - Actuator {i}: {name}")

        async with websockets.serve(handler, "localhost", 8766):
            print("WebSocket Server is actively listening on ws://localhost:8766")
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
