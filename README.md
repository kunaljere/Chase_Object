# Chase_Object
ROS 2 program that sends a TurtleBot3 Burger to track a yellow object at a set distance using proportional control

All relevant nodes are under the lilhero6_chase_object folder

**detect_object.py** - Detects an object of a specified HSV color and sends publishes the object's x,y coordinates within the Raspberry Pi V2 camera frame. 

**get_object_range.py** - Converts x,y coordinate positions into angle positions in relation to the camera frame. Receives lidar data and publishes the distance of an object at the angle position calculated. 

**chase_object.py** - Utilizes proportional control to make the TurtleBot3 follow and consistently maintain a pre-defined distance away from the object it is tracking as the object moves. 
