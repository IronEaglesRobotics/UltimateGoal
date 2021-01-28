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
    private OpenCvCamera webcam;
    private CVPipeline pipeline;
//    private StarterStackDetector.StarterStack[] stacks;
    private StarterStackDetector.StarterStack stack;

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
                robot.drive.setInput(0, 0, -(Math.max((degrees-current)/degrees*0.5,0.1)));
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
        this.sleep(1200);
    }

    public void placeGoal() {
        robot.arm.setArm(true);
        while(robot.arm.isBusy() && opModeIsActive()) {
            sleep(1);
        }

        robot.arm.setClaw(true);
        sleep(1000);

        move(2, 0.5);

        robot.arm.setArm(false);
        while(robot.arm.isBusy() && opModeIsActive()) {
            sleep(1);
        }
    }

    @Override
    public void runOpMode() {

        robot = new Robot(hardwareMap);
        robot.setTfodZoom(2);

//        stacks = new StarterStackDetector.StarterStack[10];

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (!(isStarted() || isStopRequested())) {
//            int stackCounter = 0;
//            stacks[stackCounter] = robot.checkStack();
//            if (stackCounter == stacks.length-1) {
//                stackCounter = 0;
//            } else {
//                stackCounter++;
//            }
            idle();
        }

        //start
//        int none = 0;
//        int one = 0;
//        int four = 0;
//        for (StarterStackDetector.StarterStack possibleStack : stacks) {
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
        stack = robot.checkStack();
        robot.shutdownVuforia();
        telemetry.addData("Stack:", stack);
        telemetry.update();

        // start up the second camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        this.pipeline = new CVPipeline();
        webcam.setPipeline(pipeline);

        // create asynchronous camera stream in the app using EasyOpenCV.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        // secure wobble goal
        robot.arm.setArm(false);
        robot.arm.setClaw(true);

        // everything below this should be the correct steps but has not been tested yet!!!

        // move up to white line and shoot 3 rings into the high goal
        move(24,0.4);
        strafe(12,0.4);
        robot.shooter.setShooter(0.630);
        move(40,0.4);
        strafe(-12,0.4);
        shoot();
        shoot();
        shoot();
        robot.shooter.setShooter(0);
        turn(178);

        // move to drop off wobble goal in correct location and move back to the white line
        switch(stack) {
            case NONE:
                strafe(-10,0.4);
                placeGoal();
                strafe(12,0.4);
                move(-6,0.4);
                break;
            case SINGLE:
                move(-36,0.4);
                placeGoal();
                move(6,0.4);
                break;
            case QUAD:
                move(-48,0.4);
                strafe(-10,0.4);
                placeGoal();
                strafe(10,0.4);
                move(40,0.4);
        }
        turn(178);

        telemetry.addData("Status", "finished");
        telemetry.update();
    }
}