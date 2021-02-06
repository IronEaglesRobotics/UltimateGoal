# Iron Eagles Robotics 3708 Teamcode

## To connect to the robot over wifi:
1. first connect to the ftc-prn network (password is password)
1. then run 'adb connect 192.168.43.1:5555' in the terminal (you may have to specify the specific path for where adb is located)

## How the code is structured:
* In the opmodes folder there are 4 different opmodes, 2 autonomous and 2 manual ones. The ones called sandboxes are basically duplicates that can be used to test out new functions like auto aiming before being implemented in the main opmode
* The opmodes use the classes that make up the robot from the robot folder to direct the robot to make certain movements
* the opencv folders contains classes for detection of something that our code recognizes as something, and the pipelines are just methods that run every time a frame is received from the cameras
* the CVHelpers and MathHelpers contain helper methods for the rest of the code
* the constants class contains a list of all the major constants that don't change

## Some important terms that could be helpful to know:
* HardwareMap ... A list of the parts on a robot, like a phone book for motors and servos
* Contour ... An outline, especially one bounding a shape. In this case, it is whatever OpenCV thinks is a shape
* Point ... A point on the image. Note: x=0 and y=0 start in the *top left*
* Imgproc ... The core OpenCV instance
* Moments ... The average of all grayscale values in an image. Used for finding the center of an image. See: https://www.learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/
* Mat ... Matrix, what you store images in. Named such because an image is just a matrix of pixels
* MatOfInt, MatOfFLoat, MatOf[Type] ... The Mat data type but the cells store data in a certain way
* Mask ... The isolation of a certain range of colors
* HSV ... Another way of representing color digitally. The standard way is RGB
* "Detection" ... The detection class that we built. Is initialized with two variables [see definition]
* MatOfPoint ... Pay extra attention to this type of MatOf[Type]. This is what we use to store contours
* Erode ... To thin the shapes that make up a mask. OpenCV docs give examples of this
* Dilate ... Opposite of erode
* "extends OpenCvPipeline" ... This is important. This pipeline class extends FTC's pipeline class. To use EasyOpenCV you need to specify a pipeline, this is ours
* OpenCvCameraFactory ... From FTC's EasyOpenCV. Makes it easier to get camera IDs
* OpenCvCamera ... Another camera extension from FTC robotics
* Mecanum ... Mecanum wheels are those cool omnidirectional wheels we use
* Ticks ... the smallest distance you can possibly move the wheel
* Vector ... A term to know if you don't already. Suggested: https://www.youtube.com/watch?v=fNk_zzaMoSs
* OpMode ... A series of commands for the robot to follow. Can be interchanged in the control app
* Iterative*  OpMode (or just "OpMode") ... A way to run your code. Abstracts code into stop(), start(), init(), init_loop(), loop() functions. Very often used for TeleOp since it allows asynchronous programming
* Linear OpMode ... Another way to run your code. Code is executed linearly and synchronously, just like normal. This is often used for autonomous because it is very straightforward
* IMU ... Inertial Measurement Unit. Measures an object's change in motion, among other things
* BNO055IMU ... A type of*  IMU sensor from Bosch hardware
* Telemetry ... Recording and or transmission of the readings of an instrument. Basically: environment sensing
* STRUCTURING_ELEMENT ... The shape that controls dilation of an image. Can be a star, ellipse, square, or etc
* RunMode ... The way the wheels are set to run. A few examples are: RUN_USING_ENCODER (run wheels and collect data), STOP_AND_RESET_ENCODER (brake and discard encoder data), RUN_TO_POSITION (drive until you reach a certain position)
* Encoder ... A thing that collects and stores data about hardware. In our case: wheel data

## How to add on your own code:
1. download android studio and clone (copy) this current git repository to somewhere local on your computer
1. ask Senor Scott or Mr. Thurlow to give you your own branch if you don't already have one so we don't all try to change the same branch of code
1. add your own code (try to add comments, make it readable, and make it something that you think will work / be an improvement over what was there before)
1. go to VCS and click the commit button, add your commit message (what did you change), then select your branch at the bottom and click push
1. remember to just commit to your own branch unless you have permission otherwise, feel free to merge other branches into your own or suggest code that you think other people should include in their branches
1. overall have fun and enjoy coding!