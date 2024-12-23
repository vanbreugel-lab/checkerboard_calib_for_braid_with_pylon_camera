# Guide for performing a checkerboard calibration using ROS and applying to BRAID

For Basler USB 3 cameras running on ROS Noetic in Ubuntu 20. 

## Getting pylon ros camera running

This works best (with Braid) if you use the old pylon_camera package. See instructions for [ubuntu 20 install nodes](https://github.com/vanbreugel-lab/wind_tunnel_resources/blob/main/installation/ubuntu_20_noetic_install_notes.md). If you try with the newer pylon-ros-camera package, you will probably have problems. 

## Assign device IDs

This is useful and necessary to use ros pylon_camera when there are multiple cameras plugged in. One at a time plug in a USB camera, use pylon viewer (`/opt/pylon/bin/pylonviewer`) to check the device serial number, then run the following command to set the device id. Then you can use this device id to specify which camera to launch with pylon_camera.

`rosrun pylon_camera write_device_user_id_to_camera 40450773`

Then __unplug and replug your USB camera__!

![alt_text](images/write_device_id.png "write device id")

## Have checkerboard calibrator on hand. 

We use a IR backlit printed sheet. 

![alt_text](images/checkerboard.jpg "edit yaml")


## Run pylon_camera and ROS checkerboard for each camera

To make this easy you can use the `.yaml` and `.launch` files in the `pylon_checker_launch` directory. 
1. Edit `checkerboard_cal_config.yaml` so that the `device_user_id` matches the device id o/f the camera that you want to calibrate. See screen shot below.
2. If needed, edit `checkerboard_cal_config.yaml` so that the `exposure` gives you a good checkerboard image
3. Run the launch file: `roslaunch checker_board_cal.launch`. *NOTE*: you may need to edito the launch file to ensure that the number of checkers and size is correct! This launch file is hard coded for our checkerboard calibrator. 
4. Move the checkerboard around until the user interface shows green lines for x, y, and skew (and is possible size, but that does not always happen). Once you have enough data the calibrate button will be green. See screen shot below. 
5. Press calibrate.
6. Go to the terminal where you ran the launch file. It will (after some computation) print out all the relevant calibration data. Copy the text, starting at "mono pinhole calibration..." and paste it into a text file, and save it with the full device number (e.g. 40196688.txt). See screen shot below. 
7. Repeat for every device id. 
8. Once complete you should have a directory that looks a lot like the one in `test_data`

## Sanity check your checkerboard calibration values

1. The checkerboard calibration yields a intrinsic camera matrix:

[fx, 0, px]
[0, fy, py]
[0, 0, 1]

  * fx and fy are the focal length. For standard lenses, these should be quite similar in value. 
  * px and py are the camera center. These should be close to 1/2 of the camera resolution in the x and y dimensions.

2. The checkerboard calibration also yields four distortion parameters.

  * the distortion parameters should be fairly small if you using standard (non fisheye) lenses. 

#### 1. Edit the `device_user_id` in `checkerboard_cal_config.yaml`
![alt_text](images/ros_checkerboard_calibration_yaml.png "edit yaml")

#### 4. Checkerboard GUI after collecting enough data
![alt_text](images/ros_checkerboard_calibration_gui.png "checkerboiard")

#### 6. Terminal print out
![alt_text](images/ros_checkerboard_calibration_output.png "terminal print out")

## Convert raw ROS text outputs to yaml files compatible with BRAID

Edit `convert_raw_ros_to_yaml.py`, in particular these lines:
```
    source_dir = 'test_data'
    dest_dir = 'test_data_yamls'
    camera_basename = 'Basler-'
```

Run the python script: `python convert_raw_ros_to_yaml.py`. It should create and populate the `dest_dir` with yaml files. You are now ready to use the checkerboard calibration files in BRAID. 
