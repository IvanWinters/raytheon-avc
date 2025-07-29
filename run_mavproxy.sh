#!/bin/bash

log_file="/home/$(whoami)/Desktop/mavproxy_logs/mavproxy_log_$(date -Iseconds).log"

echo "$log_file created"

sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyUSB*
mavproxy.py --master=/dev/ttyACM0 --baudrate 115200 --out=udp:127.0.0.1:14550 |& tee $log_file
