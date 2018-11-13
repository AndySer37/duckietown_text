#!/bin/bash
!/bin/bash

set -e

source /home/software/docker/env.sh

roslaunch text_navigation cnn_mser_text.launch veh:=$HOSTNAME

