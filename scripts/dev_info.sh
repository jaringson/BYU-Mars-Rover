#!/bin/bash
udevadm info -a -p $(udevadm info -q path -n $1)
