from pymavlink import mavutil

# Step 1: connect

# Start a connection (change port if needed)
connection  = mavutil.mavlink_connection('tcp:127.0.0.1:5762')

# Wait for the first heartbeat
print("Waiting for heartbeat...")
connection .wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (connection .target_system, connection .target_component))

# Step 2: send waypoint
# Target parameters for the waypoint
lat = -34 * 1e7     # Target latitude in degrees, scaled to 1e7
lon = 150 * 1e7      # Target longitude in degrees, scaled to 1e7
alt = 10                  # Target altitude in meters

# Send the waypoint
connection.mav.mission_item_int_send(
    connection.target_system,                # Target system ID
    connection.target_component,             # Target component ID
    0,                                       # Sequence number (0 for the first waypoint)
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Frame
    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,           # Command: NAV_WAYPOINT
    0,                                        # Current waypoint (set to 0 if this isn't the current waypoint)
    1,                                        # Autocontinue (1 to continue to the next waypoint)
    0,                                        # Hold time in seconds (0 for no delay)
    0,                                        # Acceptance radius in meters (0 for default)
    0,                                        # Pass radius in meters (0 to stay on the waypoint)
    0,                                        # Yaw angle (0 for default heading)
    int(lat),                                 # Latitude (scaled by 1e7)
    int(lon),                                 # Longitude (scaled by 1e7)
    int(alt * 1000)                           # Altitude in millimeters
)
print("Waypoint sent.")

# # Set the vehicle to AUTO mode
# connection.mav.set_mode_send(
#     connection.target_system,
#     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
#     3  # 3 is typically the custom mode for AUTO in ArduPilot
# )

# print("Vehicle set to AUTO mode.")