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
    OpenCvCamera webcam;
    CVPipeline pipeline;

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

    public void turn(int degrees, double power) {
        final float fudge = 7;
        if (degrees > 0) {
            while (robot.getGyroHeading360() < degrees-fudge) {
                robot.drive.setInput(0, 0, -power);
            }
        } else {
            while (robot.getGyroHeading360() > 360-degrees+fudge) {
                robot.drive.setInput(0, 0, power);
            }
        }
    }
    public void shoot(double power) {
        robot.shooter.setPusher(true);
        this.sleep(500);
        robot.shooter.setPusher(false);
        this.sleep(1000);
    }

    public void placeGoal() {
        robot.arm.setArm(true);
        while(robot.arm.isBusy() && opModeIsActive()) {
            sleep(1);
        }

        robot.arm.setClaw(true);
        sleep(1000);

        move(-2, 0.5);

        robot.arm.setArm(false);
        while(robot.arm.isBusy() && opModeIsActive()) {
            sleep(1);
        }
    }

    @Override
    public void runOpMode() {

        robot = new Robot(hardwareMap);

        robot.setTfodZoom(2);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (!(isStarted() || isStopRequested())) {
            idle();
        }

        //Check the stacks of rings and shut down the vuforia camera
        StarterStackDetector.StarterStack stack = robot.checkStack();
        robot.shutdownVuforia();
        telemetry.addData("Stack:", stack);
        telemetry.update();

        //Start up the second camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        this.pipeline = new CVPipeline();
        webcam.setPipeline(pipeline);

        //Create asynchronous camera stream in the app using EasyOpenCV.
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
        robot.arm.setClaw(false);

        // move forward and shoot 3 rings into the high goal
        robot.shooter.setShooter(0.635);
        move(24, 0.4);
        shoot(0.63);
        shoot(0.63);
        shoot(0.63);
        robot.shooter.setShooter(0);

        // TO-DO: move around the starting stack of rings and place wobble goal, then move back to park in the start line
//        switch(stack) {
//            case NONE:
//                strafe(22, 0.5);
//                move(48, 0.5);
//                placeGoal();
//                move(-6, 0.5);
//                strafe(-24, 0.5);
//                break;
//            case SINGLE:
//                strafe(22, 0.5);
//                move(70, 0.5);
//                strafe(-24, 0.5);
//                placeGoal();
//                move(-24, 0.5);
//                break;
//            case QUAD:
//                strafe(8, 0.5);
//                move(96, 0.5);
//                placeGoal();
//                move(-48, 0.5);
//                strafe(-24, 0.5);
//        }

        telemetry.addData("Status", "finished");
        telemetry.update();
    }
}