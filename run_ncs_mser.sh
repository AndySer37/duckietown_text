#!/bin/bash
!/bin/bash

set -e

source /home/duckietown_text/docker/env.sh

roslaunch text_navigation cnn_mser_text.launch veh:=$HOSTNAME

