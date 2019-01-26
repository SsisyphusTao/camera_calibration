# camera_calibration
This is a demo using OpenCV to calibrate camera and save params as .yaml file.

In 'pictures' folder 20 pictures of 4 x 11 circle pattern provided to calculate `cameraMatrix`.

>Please using `roslaunch` to run nodes else the pictures cannot be loaded
>
>roslaunch camera_calibration camera_calibration.launch
>
>rosservice call /calibrate

|Photoing Phone|Image Size|
|-|-|
|SAMSUMG NOTE 8|4032x1960|

Remaining problems:
1. `findCirclesGrid` only works when `pyrDown` executed three times which means images are 1/64 as big as origin(same  in using `resize`).

2. some centers of some pictures are not accurate.
##### Both problems may be solved by adjusting `Params` of `SimpleBlobDetector` and `TermCriteria` of `cornerSubPix`.


---------
For more details: [Calibrate_camera_intrinsics_with_circle_pattern](https://github.com/SsisyphusTao/camera_calibration/blob/master/Calibrate_camera_intrinsics_with_circle_pattern.pdf)
