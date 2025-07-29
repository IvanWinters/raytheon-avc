#!/bin/bash

# 1st tab: Run 1 MAVProxy and SITL instance for Scout
# 2nd tab: Run 1 other MAVProxy and SITL instance for Delivery Drone

# 3rd tab: Gazebo world simulation

# 4th tab: For Scout autonomous mission
# 5th tab: For Delivery autonomous mission

#gnome-terminal --tab --title="Scout: SITL" -- bash -c "cd ~/ardupilot/Tools/autotest && ./sim_vehicle.py -w -v ArduCopter -f gazebo-iris --console --map -L KSFO --instance 1 --sysid 1 --out udp:127.0.0.1:14551"
# Port 14551 for Scout Drone, Port xxx for Ground Control

#gnome-terminal --tab --title="Scout SITL" -- bash -c "cd ~/ardupilot/Tools/autotest && ./sim_vehicle.py -w -v ArduCopter -f gazebo-iris --console --map --out udp:127.0.0.1:14551"
#gnome-terminal --tab --title="Gazebo Capstone World" -- bash -c "gazebo --verbose ~/ardupilot_gazebo/worlds/capstone.world"
#gnome-terminal --tab --title="Scout: Mission" -- bash -c "python Auto_scripts/Challenge/challenge1.py --connect udpin:localhost:14551; exec bash"

gnome-terminal --tab --title="Scout SITL" -- bash -c "cd ~/ardupilot/Tools/autotest && ./sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I0 --out=udp:127.0.0.1:14551"

gnome-terminal --tab --title="Delivery SITL" -- bash -c "cd ~/ardupilot/Tools/autotest && ./sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I1 --out=udp:127.0.0.1:14552"

#gnome-terminal -tab --title="Delivery SITL" -- bash -c "cd ~/ardupilot/Tools/autotest && ./sim_vehicle.py -w -v ArduCopter -f gazebo-iris --console --map --instance 2 --sysid 2 --out=udpin:0.0.0.0:14552"

#gnome-terminal --tab --title="Delivery: SITL" -- bash -c "cd ~/ardupilot/Tools/autotest && ./sim_vehicle.py -w -v ArduCopter -f gazebo-iris --console --map -L 35.619373,-123.376637,5.3,118 --instance 2 --sysid 2 --out udp:127.0.0.1:14552"

#gnome-terminal --tab --title="Delivery: MAVProxy & SITL" -- bash -c "cd ~/ardupilot/Tools/autotest && ./sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map --out udp:127.0.0.1:14552 -I1"
# Port 14552 for Delivery Drone, Port xxx for Ground Control (e.g. 14550 or 14553-14559)

#gnome-terminal --tab --title="Delivery Console" -- bash -c "python Auto_scripts/Challenge/delivery.py --connect udpin:localhost:14552; exec bash"

#gnome-terminal --tab --title="Delivery: Mission" -- bash -c "python3.8 speed_test.py --connect udpin:localhost:14552; exec bash"