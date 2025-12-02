# YOLO Detection 
## Overview
This package uses YOLO to independently detect SAM and buoy. It includes two trained models: one for simulation and another for real-world scenarios (*.pt* files). The models can be accessed [here](https://kth-my.sharepoint.com/:f:/g/personal/framir_ug_kth_se/IgD4flS-1Nx9SoAHoldjgDS8AULieuNh-fLqTrC9wWTSeBY?e=PAd1AX). Download the models you want and change the path parameter in the *.yaml* file in config.

> Rename the model file to `yolo_model.pt` and place it inside the config directory of this package.

Two different datasets were used, with ~450 labelled images in total. Access them [here](https://kth-my.sharepoint.com/:f:/g/personal/framir_ug_kth_se/EpHV7UF6nQVIsYwrSBDlYWkBR-Yv08Lia9hxuD-aqrMTJQ?e=cpmczE). The detection of SAM's head is done with a Canny edge detector, which relies on the existence of a rope attached to it. 
### Training
The sim model was trained with the sim dataset, which was later used to train the real model with the real dataset.

## Dependencies (dev versions)
- ROS2 Humble
- Ultralytics: 8.3.160
- Numpy: 1.23.5
- OpenCV: 4.11.0
- Torch: 2.6.0
  
When installing *Ultralytics*, corresponding dependencies (*eg*: torch, opencv, etc) will be installed (check [here](https://docs.ultralytics.com/quickstart/) for more info)


> It is likely that if you run `pip3 install ultralytics` it will also install `numpy 2.2.6` (as of 2025 Nov). This is very likely to cause issues with the `tf_transformations` library (that everyone is using...). So remove numpy with `pip` if this is the case.

## Launch yolo detector (example)
Launch yolo detector:
```
ros2 launch auv_yolo_detector yolo_detector_launch.py namespace:=M350 device:=cpu use_sim_time:=true mode:=sim
```

Launch rviz file (needs absolute path):
```
rviz2 -d <your path>/perception/alars/auv_yolo_detector/config/M350_yolo.rviz 
```

Launch rosbag (for frame collection or testing purposes):
ros2 bag play --read-ahead-queue-size 1000 -r 1.0 --clock 100 --start-paused <rosbag path>
```

## **Visualization Topics**
| Topic | Msg | Description |
| --- | ---| --- |
| /namespace/rviz/annotated_image | Image | Image from camera with YOLO annotations (bounding boxes + probabilities)|
| /namespace/rviz/bw_blurred_sam  | Image | Sliced and rotated window with sam. It's in grayscale and it's blurred, since it's the input to the Canny edge detector. |
| /namespace/rviz/edges  | Image | Output from Canny edge detector|
---
The last two topics may help in debugging, since the variance parameter (detection.blur_variance) in the filter definition might have to be changed. The waves shouldn't yield any edges, so one should choose the smallest variance that makes this possible.

## Future work
- Convert bounding boxes+orientation to pose (not position)
- Estimate pose/position correctly in real scenario (not only sim)
- Implement some kind of filter or atleast outlier removal.

## Outro
Don't forget to
```
colcon build --symlink-install --packages-select auv_yolo_detector
source install/setup.sh
```
## Maintainer
Francisco Miranda, framir@kth.se
