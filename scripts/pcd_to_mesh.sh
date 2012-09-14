#!/bin/bash

source $(rospack find rosx)/bashrc_setup.sh

CLOUD_FILE=$1
shift

( sleep 2.5; rosrun_in electric pcl_ros pcd_to_pointcloud $CLOUD_FILE 0; ) & rosrun google_goggles point_cloud_to_mesh $@

