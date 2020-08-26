This is the code for the indicator lighting.

It shall listen for the topics
- /lighting/indicator/{left, right}
- /lighting/{brake, headlight}       #Note: Headlight turns on front and rear light
- /lighting/party

all as std_msgs::Bool

start listener on remote with
`rosrun rosserial_python serial_node.py tcp 11411`

If not yet configured, it will open an SSID "pilsbot_lights" with the default password `test1234`.
Static IP is not supported anymore.
