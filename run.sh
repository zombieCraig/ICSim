#!/bin/bash

trap 'kill $(jobs -p)' EXIT

cd ICSim
./controls vcan0 &         # dashboard
./icsim vcan0 &      # joystick

wait
