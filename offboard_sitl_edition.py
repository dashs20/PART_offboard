import time
from pymavlink import mavutil
import pandas as pd
import math
import numpy as np

# Connect to the vehicle
connection = mavutil.mavlink_connection('tcp:127.0.0.1:5762')

# Wait for the first heartbeat to confirm connection
print("Waiting for heartbeat...")
connection.wait_heartbeat()
print(f"Heartbeat received from system (system {connection.target_system} component {connection.target_component})")

# Initialize variables
mission_items = []
current_seq = 0  # Sequence number of the current waypoint
waypoint_reached = False

# Placeholder DataFrame to simulate receiving waypoints
def get_new_waypoint():
    # This function simulates receiving a new waypoint
    # In practice, this would be replaced by code that receives data from 0MQ
    # For the simulation, we will create a new waypoint every 10 seconds
    time.sleep(10)
    data = {
        'lat': [-35.0+(-0.5+np.random.rand(1))],            # Latitude in degrees
        'lon': [150.0+(-0.5+np.random.rand(1))],            # Longitude in degrees
        'alt': [100],              # Altitude in meters
        'if_servo': [1],           # Boolean flag for servo action
        'servo_id': [1]            # Servo ID
    }
    df = pd.DataFrame(data)
    return df

# Function to arm the vehicle
def arm_vehicle():
    print("Arming the vehicle...")
    connection.arducopter_arm()
    # Wait until the vehicle is armed
    while True:
        msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=10)
        if msg:
            armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            if armed:
                print("Vehicle is armed.")
                break
            else:
                print("Waiting for vehicle to arm...")
        else:
            print("Waiting for heartbeat...")

# Function to change flight mode
def set_mode(mode):
    # Get the mode ID from the mode string
    mode_id = connection.mode_mapping().get(mode)
    if mode_id is None:
        print(f"Unknown mode: {mode}")
        return False

    # Set the new mode
    connection.mav.set_mode_send(
        connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    return True

# Arm the vehicle and set mode to AUTO
arm_vehicle()
if set_mode('AUTO'):
    print("Flight mode set to AUTO.")
else:
    print("Failed to set flight mode to AUTO.")

# Main loop
while True:
    # Check for new waypoint from DataFrame
    df = get_new_waypoint()  # Simulate receiving a new waypoint
    if not df.empty:
        # Extract waypoint data
        lat = df['lat'].iloc[0]
        lon = df['lon'].iloc[0]
        alt = df['alt'].iloc[0]
        if_servo = df['if_servo'].iloc[0]
        servo_id = df['servo_id'].iloc[0]

        # Prepare waypoint parameters
        seq = len(mission_items)  # Sequence number
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        current = 0  # Not the current waypoint
        autocontinue = 1
        param1 = 0  # Hold time in seconds
        param2 = 5  # Acceptance radius in meters
        param3 = 0
        param4 = float('nan')
        x = int(lat * 1e7)
        y = int(lon * 1e7)
        z = alt

        # Create the mission item
        mission_item = mavutil.mavlink.MAVLink_mission_item_int_message(
            target_system=connection.target_system,
            target_component=connection.target_component,
            seq=seq,
            frame=frame,
            command=command,
            current=current,
            autocontinue=autocontinue,
            param1=param1,
            param2=param2,
            param3=param3,
            param4=param4,
            x=x,
            y=y,
            z=z,
            mission_type=0
        )
        mission_items.append({
            'mission_item': mission_item,
            'if_servo': if_servo,
            'servo_id': servo_id
        })
        print(f"New waypoint added. Total waypoints: {len(mission_items)}")

        # Send the updated mission to the vehicle
        print("Uploading mission...")
        connection.mav.mission_count_send(
            connection.target_system,
            connection.target_component,
            len(mission_items)
        )

        # Send each mission item
        for item in mission_items:
            while True:
                msg = connection.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'], blocking=True, timeout=10)
                if msg and msg.seq == item['mission_item'].seq:
                    connection.mav.send(item['mission_item'])
                    print(f"Sent waypoint {item['mission_item'].seq}")
                    break
                else:
                    print(f"Waiting for MISSION_REQUEST for seq {item['mission_item'].seq}...")

        # Wait for MISSION_ACK
        while True:
            msg = connection.recv_match(type='MISSION_ACK', blocking=True, timeout=10)
            if msg:
                if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    print("Mission acknowledged and accepted.")
                    break
                else:
                    print(f"Mission not accepted. ACK type: {msg.type}")
                    break
            else:
                print("Waiting for MISSION_ACK...")

    # Get updated parameters from the plane
    # Request data streams
    connection.mav.request_data_stream_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        4,  # Rate (Hz)
        1    # Start
    )

    # Receive and print data
    msg = connection.recv_match(type=['GLOBAL_POSITION_INT', 'ATTITUDE', 'VFR_HUD', 'HEARTBEAT'], blocking=True, timeout=1)
    if msg:
        msg_type = msg.get_type()
        if msg_type == 'GLOBAL_POSITION_INT':
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000  # Convert mm to meters
            print(f"GPS Location: Lat={lat:.7f}, Lon={lon:.7f}, Alt={alt:.2f}m")
        elif msg_type == 'ATTITUDE':
            roll = math.degrees(msg.roll)
            pitch = math.degrees(msg.pitch)
            yaw = math.degrees(msg.yaw)
            print(f"Attitude: Roll={roll:.2f}, Pitch={pitch:.2f}, Yaw={yaw:.2f}")
        elif msg_type == 'VFR_HUD':
            airspeed = msg.airspeed
            print(f"Airspeed: {airspeed:.2f} m/s")
        elif msg_type == 'HEARTBEAT':
            mode = mavutil.mode_string_v10(msg)
            print(f"Flight Mode: {mode}")

    # Check if waypoint is reached
    msg = connection.recv_match(type='MISSION_ITEM_REACHED', blocking=True, timeout=1)
    if msg:
        seq = msg.seq
        print(f"Waypoint {seq} reached.")
        # Check if we need to activate servo
        if seq < len(mission_items):
            item = mission_items[seq]
            if item['if_servo']:
                servo_id = item['servo_id']
                # Command to activate the servo (e.g., set servo to 90 degrees)
                pwm = 1900  # Example PWM value for 90 degrees
                connection.mav.command_long_send(
                    connection.target_system,
                    connection.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                    0,
                    servo_id,    # Servo number
                    pwm,         # PWM value
                    0, 0, 0, 0, 0)
                print(f"Activated servo {servo_id} at waypoint {seq}.")
    # Check if mission is completed
    msg = connection.recv_match(type='MISSION_CURRENT', blocking=True, timeout=1)
    if msg:
        current_seq = msg.seq
        if current_seq >= len(mission_items):
            print("Mission completed. Switching to LOITER mode.")
            set_mode('LOITER')
            break  # Exit the loop or continue as per requirements

    # Small delay to prevent CPU overuse
    time.sleep(0.1)
