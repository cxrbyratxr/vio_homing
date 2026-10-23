#!/bin/bash
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
manual_check /usb_camera
exec roslaunch --wait vio_homing manual_camera.launch camera_device:=/dev/v4l/by-id/usb-Global_Shutter_Camera_Global_Shutter_Camera_01.00.00-video-index0 camera_fourcc:=YUYV camera_grayscale:=false "$@"
