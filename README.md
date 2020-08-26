This is the code for the indicator lighting.

It shall listen for the topics
- /lighting/indicator/{left, right}
- /lighting/{brake, headlight}       #Note: Headlight turns on front and rear light
- /lighting/party

all as std_msgs::Bool

This node takes ip 192.168.4.5 and expects ROS master at 192.168.4.1
start listener on remote with

rosrun rosserial_python serial_node.py tcp 11411
