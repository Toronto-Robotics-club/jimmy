# This message type is used as a way for other nodes to request the LMS system
# to perform a motion. It is very basic at this point as I don't know how this
# system is going to be used down the road.

# an int value to represent direction, where:
# 1 == CW
# 2 == CCW
int32 direction

# an int value to represent the number of encoder pulses we want the LMS to
# move. A full revolution is 32,000 pulses.
int64 encPulsesToMove

# In the future, I may want to implement the following:
# 1. a way to identify requests to let the requesting node know if the request
#    is going to be executed or not (as LMS may be in motion, or down, etc...)
# 2. Implement the ability to move to an absolute position by passing a rad 
#    value as the absolute position.


