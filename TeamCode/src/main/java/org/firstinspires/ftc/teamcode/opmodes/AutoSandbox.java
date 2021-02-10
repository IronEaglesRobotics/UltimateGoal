package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;

// Sandbox Autonomous Program
@Autonomous(name = "Auto Sandbox")
public class AutoSandbox extends LinearOpMode {
    private Robot robot;
    private Constants.StarterStack stack;

    // Move in 2 dimensions
    public void move(double x, double y, double power, int state) {
        robot.drive.setTargetPositionRelative(x, y, power);
        while(robot.drive.isBusy() && opModeIsActive()) {
            sleep(1);
        }
    }

    // Turn
    public void turn(double degrees) {
        final float fudge = 2;
        degrees = Math.abs(degrees);
        robot.imu.resetGyroHeading();
        float current = 5;

        if (degrees > 0) {
            while (current < degrees-fudge || current > 360 - fudge) {
                robot.drive.setInput(0, 0, -(Math.max((degrees-current)/degrees*0.65,0.2)));
                current = robot.imu.getGyroHeading360();

                telemetry.addData("Status", current);
                telemetry.update();
                sleep(1);
            }
        }
        robot.drive.setInput(0,0,0);
    }

    // Shoot a ring
    public void shoot() {
        robot.shooter.setPusher(Constants.ServoPosition.CLOSED);
        this.sleep(500);
        robot.shooter.setPusher(Constants.ServoPosition.OPEN);
        this.sleep(500);
    }

    // Place down the goal
    public void placeGoal() {
        robot.arm.setArm(Constants.ArmPosition.DOWN);
        while(robot.arm.isBusy() && opModeIsActive()) {
            sleep(1);
        }

        robot.arm.setClaw(Constants.ServoPosition.OPEN);
        sleep(500);

        move(0, -2, 0.5, 1);

        robot.arm.setArm(Constants.ArmPosition.UP);
        while(robot.arm.isBusy() && opModeIsActive()) {
            sleep(1);
        }
    }

    // Main method to run all the steps for autonomous
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing Robot");
        telemetry.update();
        robot = new Robot(hardwareMap);
        robot.camera.initStackCamera();

        // wait for the first frame to make sure it doesn't try to analyze a non existent frame
        while (robot.camera.getFrameCount() < 1) {
            this.sleep(1);
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // wait for start
        while (!(isStarted() || isStopRequested())) {
            stack = robot.camera.checkStack();
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Stack", stack);
            telemetry.addData("Size", robot.camera.getStarterStack().getArea());
            telemetry.update();
        }

        telemetry.addData("Stack", stack);
        telemetry.addData("Ticks per rev", robot.drive.getTicksPerRev());
        telemetry.update();

        robot.camera.stopStackCamera();

        // movements for auto

        robot.camera.initTargetingCamera();

        this.sleep(2000);

        robot.camera.stopTargetingCamera();
        telemetry.addData("Status", "Finished");
        telemetry.update();
    }
}
