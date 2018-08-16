# Represents a state space.

# General info.
Header header

# Identifying string for this state space. (ID ?)
string name

# State Space type: controls how the state space is displayed / computed.
# Default: ORDINARY
# ORDINARY : General display / compute of the state space without any assumption.
# STATE_2D : Expect a 2D position and orientation (Dimension = 3).
# STATE_3D : Expect a 3D position and orientation (quaternion) (Dimension = 7).
# STATE_BOOL : Expect a single boolean information (ie on/off button) (Dimension = 1).
uint8 ORDINARY=0
uint8 STATE_BOOL=1
uint8 STATE_2D=2
uint8 STATE_3D=3

uint8 state_type

#Objective type: boolean (true : Just interact with the object without taking in account State Space data)
uint8 objective_type

#Maximum distance error allowed arround the objective
float32 max_error

#State Space data : contains the state space data.
uint8[] discrete_data
float32[] real_data

# Short description (< 40 characters) of what this state space represent,
# e.g. "Button A pressed". 
# Default: ?
string description
