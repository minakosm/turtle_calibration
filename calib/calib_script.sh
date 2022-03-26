#! /bin/bash

ros2 run turtle_camera singleCapture

echo "In order to run calibration insert the following required arguments

--image_dir: Image Directory
--image_format: png/jpg
--translation: x y z translation vector

Default arguments: --square_size	 (11)
		   --width		 (4)
		   --height		 (6)
		   --intrinsic_calib	 (0)

If you want to perform intrinsic calibration type --intrinsic_calib 1
"
read ARGUMENTS

ros2 run turtle_calibration calibration $ARGUMENTS




