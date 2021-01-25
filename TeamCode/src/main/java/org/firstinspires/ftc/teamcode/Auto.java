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

    Detection red = null;
    Detection blue = null;

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
        robot.drive.setPower(0);
        this.sleep(2000);
        telemetry.addData("", robot.getGyroHeading360());
        telemetry.update();
        this.sleep(10000);
    }

    public void placeGoal() {
        robot.arm.setTargetArmPosition(120, 0.5);
        while(robot.arm.isBusy() && opModeIsActive()) {
            sleep(1);
        }

        robot.arm.setClaw(true);
        sleep(1000);

        move(-2, 0.5);

        robot.arm.setTargetArmPosition(-120, 0.5);
        while(robot.arm.isBusy() && opModeIsActive()) {
            sleep(1);
        }
    }

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);

        robot.setTfodZoom(3);
//        robot.arm.setClaw(false);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        while (!(isStarted() || isStopRequested())) {
            idle();
        }

        //Check the stacks of rings and shut down the vuforia camera
        StarterStackDetector.StarterStack stack = robot.checkStack();
        robot.shutdownVuforia();

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

        move(6,0.5);
        strafe(10,0.5);

        switch(stack) {
            case NONE:
                strafe(22, 0.5);
                move(48, 0.5);
                placeGoal();
                move(-6, 0.5);
                strafe(-24, 0.5);
                break;
            case SINGLE:
                strafe(22, 0.5);
                move(70, 0.5);
                strafe(-24, 0.5);
                placeGoal();
                move(-24, 0.5);
                break;
            case QUAD:
                strafe(22, 0.5);
                move(96, 0.5);
                placeGoal();
                move(-48, 0.5);
                strafe(-24, 0.5);
        }
        red = pipeline.getRed();
        blue = pipeline.getBlue();

        double x = 0;
        double y = 0;
        double z = 0;

        double zErr = Math.abs(red.getCenter().x);

        // Aim for goal
        while(zErr > 1) {
            double zMaxSpeed = 0.7;
            zErr = Math.abs(red.getCenter().x);
            double zSpeed = (zErr / 50) * zMaxSpeed;

            if (zErr <= 1) {
                z = 0;
            } else if (zErr > 1) {
                z = Math.copySign(zSpeed, red.getCenter().x);
            }

            //                double yMaxSpeed = 0.7;
            //                double yErr = Math.abs(5-red.getArea());
            //                double speed = (yErr / 5) * yMaxSpeed;
            //                if (yErr <= 0.1) {
            //                    y = 0;
            //                } else if (yErr > 0.1) {
            //                    y = Math.copySign(speed, 5-red.getArea());
            //                }
            telemetry.addData("FPS", String.format("%.1f", webcam.getFps()));
            telemetry.addData("Red", String.format("Area: %.1f, Center: (%.1f, %.1f)", red.getArea(), red.getCenter().x, red.getCenter().y));
            telemetry.addData("Blue", String.format("Area: %.1f, Center: (%.1f, %.1f)", blue.getArea(), blue.getCenter().x, blue.getCenter().y));
            telemetry.update();

            telemetry.addData("Status", "finished");
            telemetry.update();

            sleep(100);

        }
    }

}