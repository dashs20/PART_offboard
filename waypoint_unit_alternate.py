from pymavlink import mavutil
import time

# Step 1: Connect to the vehicle
connection = mavutil.mavlink_connection('tcp:127.0.0.1:5762')

# Wait for the first heartbeat to confirm connection
print("Waiting for heartbeat...")
connection.wait_heartbeat()
print("Heartbeat received from system (system %u component %u)" %
      (connection.target_system, connection.target_component))

# Optional: Clear existing missions
connection.mav.mission_clear_all_send(
    connection.target_system,
    connection.target_component
)

# Send MISSION_COUNT to inform the vehicle of the number of waypoints
connection.mav.mission_count_send(
    connection.target_system,
    connection.target_component,
    1  # Number of mission items
)

# Wait for MISSION_REQUEST or MISSION_REQUEST_INT from the vehicle
while True:
    msg = connection.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'], blocking=True, timeout=10)
    if msg and msg.seq == 0:
        break
    else:
        print("Waiting for MISSION_REQUEST...")

# Define the target waypoint in degrees
lat = -34.0     # Target latitude in degrees
lon = 150.0     # Target longitude in degrees
alt = 100       # Target altitude in meters

# Prepare waypoint parameters
seq = 0  # Sequence number
frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT  # Frame for mission_item_int
command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT  # Command type
current = 1  # Mark as current waypoint
autocontinue = 1  # Autocontinue to next waypoint
param1 = 0  # Hold time in seconds
param2 = 5  # Acceptance radius in meters
param3 = 0  # Pass radius in meters
param4 = float('nan')  # Desired yaw angle at waypoint (NaN for default)

# Scale latitude and longitude
x = int(lat * 1e7)  # Latitude scaled by 1e7
y = int(lon * 1e7)  # Longitude scaled by 1e7
z = alt  # Altitude in meters

# Print scaled values for verification
print(f"x (lat * 1e7): {x}")
print(f"y (lon * 1e7): {y}")
print(f"z (alt): {z}")

# Send MISSION_ITEM_INT with the waypoint details
connection.mav.mission_item_int_send(
    connection.target_system,        # Target system
    connection.target_component,     # Target component
    seq,                             # Sequence
    frame,                           # Frame
    command,                         # MAVLink command
    current,                         # Current waypoint
    autocontinue,                    # Autocontinue to next waypoint
    param1,                          # Hold time
    param2,                          # Acceptance radius
    param3,                          # Pass radius
    param4,                          # Yaw angle
    x,                               # Latitude
    y,                               # Longitude
    z                                # Altitude
)
print("Waypoint sent.")

# Wait for MISSION_ACK from the vehicle
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

# Arm the vehicle
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

# Set the current mission item to the first waypoint (sequence number 0)
connection.mav.mission_set_current_send(
    connection.target_system,
    connection.target_component,
    seq  # Sequence number of the waypoint to set as current
)
print("Set current mission item to waypoint 0.")

# Wait for MISSION_CURRENT message to confirm
while True:
    msg = connection.recv_match(type='MISSION_CURRENT', blocking=True, timeout=5)
    if msg:
        print(f"Current mission item is set to {msg.seq}.")
        if msg.seq == seq:
            break
    else:
        print("Waiting for MISSION_CURRENT message...")

# Change the flight mode to AUTO to start mission execution
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

# Attempt to set mode to AUTO
if set_mode('AUTO'):
    print("Flight mode change requested: AUTO.")
else:
    print("Failed to request flight mode change.")

# Confirm the mode change
while True:
    msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if msg:
        # Check if the mode is now AUTO
        mode = mavutil.mode_string_v10(msg)
        if mode == 'AUTO':
            print("Flight mode changed to AUTO.")
            break
        else:
            print(f"Current mode: {mode}. Waiting for mode change...")
    else:
        print("Waiting for heartbeat to confirm mode change...")

# Monitor for MISSION_ITEM_REACHED messages
print("Monitoring mission execution...")
while True:
    msg = connection.recv_match(type=['MISSION_ITEM_REACHED', 'STATUSTEXT'], blocking=True, timeout=30)
    if msg:
        if msg.get_type() == 'MISSION_ITEM_REACHED' and msg.seq == seq:
            print(f"Waypoint {msg.seq} reached.")
            break
        elif msg.get_type() == 'STATUSTEXT':
            print(f"Status message: {msg.text}")
    else:
        print("Waiting for mission item to be reached...")
