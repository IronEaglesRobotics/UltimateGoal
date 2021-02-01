package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/*Terms:
* - OpMode ... A series of commands for the robot to follow. Can be interchanged in the control app.
* - Iterative OpMode (or just "OpMode") ... A way to run your code. Abstracts code into stop(), start(), init(), init_loop(), loop() functions. Very often used for TeleOp since it allows asynchronous programming.
* - Linear OpMode ... Another way to run your code. Code is executed linearly and synchronously, just like normal. This is often used for autonomous because it is very straightforward.
*/

// autonomous program
@Autonomous(name = "Auto")
public class Auto extends LinearOpMode {
    private Robot robot;
    private OpenCvCamera stackCamera;
    private OpenCvCamera targetingCamera;
    private StarterStackPipeline stackPipeline;
    private TargetingPipeline targetingPipeline;

//    private StarterStackDetector.StarterStack[] stacks;
//    private StarterStackDetector.StarterStack stack;

    public void move(int inches, double power) {
        robot.drive.setTargetForwardPositionRelative(inches, power);
        while(robot.drive.isBusy() && opModeIsActive()) {
            sleep(1);
        }
    }

    public void strafe(int inches, double power) {
        robot.drive.setTargetStrafePositionRelative(inches, power);
        while(robot.drive.isBusy() && opModeIsActive()) {
            sleep(1);
        }
    }

    public void turn(double degrees) {
        final float fudge = 2;
        degrees = Math.abs(degrees);
        robot.resetGyroHeading();
        float current = 5;

        if (degrees > 0) {
            while (current < degrees-fudge || current > 360 - fudge) {
                robot.drive.setInput(0, 0, -(Math.max((degrees-current)/degrees*0.65,0.2)));
                current = robot.getGyroHeading360();

                telemetry.addData("Status", current);
                telemetry.update();
                sleep(1);
            }
        }
        robot.drive.setInput(0,0,0);
    }

    public void shoot() {
        robot.shooter.setPusher(true);
        this.sleep(500);
        robot.shooter.setPusher(false);
        this.sleep(500);
    }

    public void placeGoal() {
        robot.arm.setArm(true);
        while(robot.arm.isBusy() && opModeIsActive()) {
            sleep(1);
        }

        robot.arm.setClaw(false);
        sleep(500);

        move(2, 0.5);

        robot.arm.setArm(false);
        while(robot.arm.isBusy() && opModeIsActive()) {
            sleep(1);
        }
    }

    @Override
    public void runOpMode() {

        robot = new Robot(hardwareMap);
//        robot.setTfodZoom(2);

//        stacks = new StarterStackDetector.StarterStack[10];



//        for(int stackCounter = 0; !(isStarted() || isStopRequested()); stackCounter = (stackCounter + 1) % 10) {
//            stacks[stackCounter] = robot.checkStack();
//            telemetry.addData("",stacks[0]+" "+stacks[1]+" "+stacks[2]+" "+stacks[3]+" "+stacks[4]+" "+stacks[5]+" "+stacks[6]+" "+stacks[7]+" "+stacks[8]+" "+stacks[9]);
//            telemetry.update();
//        }

        // start up the first camera
        int stackCameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.stackCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Stack Webcam"), stackCameraMonitorViewId);
        this.stackPipeline = new StarterStackPipeline();
        stackCamera.setPipeline(stackPipeline);
        // create asynchronous camera stream in the app using EasyOpenCV.
        stackCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                stackCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (!(isStarted() || isStopRequested())) {
            idle();
        }


        //start
//        int none = 0;
//        int one = 0;
//        int four = 0;
//        for (StarterStackDetector.StarterStack possibleStack : stacks) {
//            if (possibleStack == null) {
//                continue;
//            }
//            switch(possibleStack) {
//                case NONE: none++; break;
//                case SINGLE: one++; break;
//                case QUAD: four++;
//            }
//        }
//        if (none > Math.max(one, four)) {
//            stack = NONE;
//        } else if (one > Math.max(none, four)) {
//            stack = SINGLE;
//        } else if (four > Math.max(none, one)) {
//            stack = QUAD;
//        }

        // check the stacks of rings and shut down the vuforia camera
//        stack = robot.checkStack();
//        robot.shutdownVuforia();
//        telemetry.addData("Stack:", stack);
//        telemetry.update();

        // start up the second camera
        int targetingCameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.targetingCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Targeting Webcam"), targetingCameraMonitorViewId);
        this.targetingPipeline = new TargetingPipeline();
        targetingCamera.setPipeline(targetingPipeline);
        // create asynchronous camera stream in the app using EasyOpenCV.
        targetingCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                targetingCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        // secure wobble goal

        robot.arm.setClaw(true);

        // everything below this should be the correct steps but has not been tested yet!!!

        // move up to white line and shoot 3 rings into the high goal
//        move(24,0.5);
//        robot.arm.setArm(false);
//        strafe(12,0.55);
//        robot.shooter.setShooter(0.630);
//        move(40,0.5);
//        strafe(-15,0.5);
//        shoot();
//        shoot();
//        shoot();
//        robot.shooter.setShooter(0);
//        turn(178);
//
//        // move to drop off wobble goal in correct location and move back to the white line
//        switch(stack) {
//            case NONE:
//                strafe(-8,0.5);
//                placeGoal();
//                strafe(24,0.5);
//                move(-12,0.5);
//                break;
//            case SINGLE:
//                move(-22,0.5);
//                strafe(12,0.5);
//                placeGoal();
//                move(6,0.5);
//                break;
//            case QUAD:
//                move(-46,0.5);
//                strafe(-6,0.5);
//                placeGoal();
//                move(30,0.5);
//        }

        telemetry.addData("Status", "finished");
        telemetry.update();
    }
}