from udacidrone import Drone
from udacidrone.connection.mavlink_connection import MavlinkConnection

# Initiate a Mavlink connection
conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=True)

# Initialize a drone
drone = Drone(conn)

# Start receiving messages from the drone
drone.start()

# Take control and set the mode to guided
drone.take_control()

# Arms the rotors
drone.arm()

# Set the drone's home position
drone.set_home_position(drone.global_position[0], 
                        drone.global_position[1], 
                        drone.global_position[2])

# Takeoff
drone.takeoff(3)

# Send a position (north, east, down, heading) command
drone.cmd_position(5,0,3,0)
