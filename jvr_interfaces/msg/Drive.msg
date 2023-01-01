# Message for driving. Two motors on the robot.

# Time of the message. Older messages will be ignored, 
# the motors controller will remember wat message last
# is used
builtin_interfaces/Time stamp


# Motor power (in percentage), positive for forward
# movement, negative value for backward movement
# Value between 0 and 1.0 (but less than .2 is ignored)
float32 left
float32 right

# Duration of the drive (0 for infinite)
builtin_interfaces/Duration duration