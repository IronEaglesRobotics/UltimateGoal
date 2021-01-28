 package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

 //Sandbox auto program
@Autonomous(name = "AutoSandbox")
public class AutoSandbox extends LinearOpMode {
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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        this.pipeline = new CVPipeline();
        webcam.setPipeline(pipeline);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (!(isStarted() || isStopRequested())) {
            idle();
        }

        turn(178);

        telemetry.addData("Status", "Finished");
        telemetry.update();
    }
}