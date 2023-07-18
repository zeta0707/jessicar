#!/bin/bash

## if no parameter, then exit after print usage
if [ "$#" -lt 1 ]; then
    echo "Usage: $0 target"
    echo "target: select one between these"
    echo "csicam, usbcam"
	exit 1
fi

if [ "$1" == "usbcam" ]; then
    sed -i 's/csicam/usbcam/g' ../jessicar_control/launch/blob_all.launch
    sed -i 's/csicam/usbcam/g' ../jessicar_control/launch/traffic_all.launch
    sed -i 's/csicam/usbcam/g' ../jessicar_control/launch/yolo_all.launch
else
    sed -i 's/usbcam/csicam/g' ../jessicar_control/launch/blob_all.launch
    sed -i 's/usbcam/csicam/g' ../jessicar_control/launch/traffic_all.launch
    sed -i 's/usbcam/csicam/g' ../jessicar_control/launch/yolo_all.launch
fi  