# The raw information provided by introspective perception to 
# inform navigation of the likelihood of failure
float32 failure_likelihood
uint8 failure_type

# TODO: Introspective perception should provide the robot pose
# from where the source of failure has been detected. Currently
# navigation associates the failure likelihood to the pose of
# the robot at the time of receiving the msg but this is prone
# to error in the presense of communication delay.

# float32 loc_x
# float32 loc_y
# float32 loc_theta