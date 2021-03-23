# 3708 Iron Eagles Robotics Teamcode

## How the code is structured:  
- All of the code that we work on is underneath the teamcode folder, everything else can be thought of as magical extra files that make the app work (although reading through what is available to get an understanding of how it works is great too)
- The [drive](drive) and [util](util) folders both contain roadrunner helper files and opmodes
- In the [opmodes](opmodes) folder there are different opmodes for autonomous and driver control periods
- The [robot](robot) folder houses all the different sub classes for the robot such as the arm and intake
- The [opencv](opencv) folders contains classes for detections and pipelines that run every time a frame is received from the cameras
- The [Constants](Constants.java) class contains a list of all the major constants that don't change

## Viewing the Dashboard:
1. First connect to the FTC-prN network (password is password)
2. Go to `192.168.43.1:8080/dash` in a web browser
3. You should then be able to view the opmodes as well as view of where the robot thinks it is on the field

## Uploading code to the robot over USB-C:  
1. First connect to the robot control hub via usb-c
2. Open your code to upload in Android Studio
3. Press `Shift + F10` or click the green triangle to upload your code

## Uploading code to the robot over WI-FI:  
1. First connect to the FTC-prN network (password is password)  
2. Then run `adb connect 192.168.43.1:5555` in the terminal (you may have to specify the specific path for where adb is located)  
3. If for some reason it doesn't connect, connect via usb-c and run `adb tcpip 5555` in the terminal to restart the port in tcp mode and try again
4. Open your code to upload in Android Studio
5. Press `Shift + F10` or click the green triangle to upload your code

## Creating your own code:  
1. download android studio and clone (copy) this current git repository to somewhere local on your computer  
2. ask Senor Scott or Mr. Thurlow to give you your own branch if you don't already have one so we don't all try to change the same branch of code  
3. add your own code (try to add comments, make it readable, and make it something that you think will work / be an improvement over what was there before)  
4. press `Alt + S` and then `Ctrl + K` or click `VCS` and `Commit...` at the top
5. add your commit message (what you changed), then press `Ctrl + Alt + K` or the `Commit and Push` button to push your code
6. remember to **just commit to your own branch** unless you have permission otherwise, feel free to merge other branches into your own or suggest code that you think other people should include in their branches  
7. overall have fun and enjoy coding!