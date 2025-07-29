#!/bin/bash

echo "Starting Flight $1"

python $1 --connect 127.0.0.1:14550

echo "Flight Complete"
