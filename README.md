# hyper_drive
### Introduction to hsi_driver.py and cube_data.py
The main two code files in this repository are hsi_driver.py, and cube_data.py. hsi_driver.py is responsible for publishing images taken by the imec and ximea cameras,
and as well as for subscribing to these published images. cube_data.py also contains a publisher, and a subscriber, but instead of images, for data cubes from imec and ximea cameras.

### Camera parameter adjustments
The camera parameters can be adjusted through use of a rosservice called /adjust_param. Currently the only supported parameter adjustment is exposure time in milliseconds
When running the code, you can check to see if the srv is working by running:
> rosservice list

And you should see:
> /adjust_param

Then the value can then be adjusted by the command:
> rosservice call /adjust_param "user_in: (val)"

### Choosing camera model
The camera model can be changed by calling upon a parameter called /cube_processor/camera_model for cube_data.py, and /hsi_processor/camera_model for hsi_driver.py.
These can be changed to a string of either "imec" or "ximea" based off which camera you want the code to extract data from. 
This srv can be started by first running:

> roslaunch imec_driver camera.launch 

This starts the launch file where the user_in input can then be adjusted.
You can check to see if the params are now visible by running:
> rosparam list

Where you should see 
> /cube_processor/camera_model

> /hsi_processor/camera_model

You can check the values in the params by running
> rosparam get (param)

And the value of the parameter can be set by running
> rosparam set (param) (val)
