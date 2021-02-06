package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;

// Main Autonomous Program
@Autonomous(name = "Auto")
public class Auto extends LinearOpMode {
    private Robot robot;
    private Constants.StarterStack stack;

    // Move forward and backward a certain number of inches
    public void move(int inches, double power) {
        robot.drive.setTargetForwardPositionRelative(inches, power);
        while(robot.drive.isBusy() && opModeIsActive()) {
            sleep(1);
        }
    }

    // Move sideways a certain number of inches
    public void strafe(int inches, double power) {
        robot.drive.setTargetStrafePositionRelative(inches, power);
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

        move(2, 0.5);

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
            telemetry.update();
        }

        telemetry.addData("Stack", stack);
        telemetry.update();

        robot.camera.stopStackCamera();

        // movements for auto

        // secure wobble goal
        robot.arm.setClaw(Constants.ServoPosition.CLOSED);
        robot.camera.initTargetingCamera();

        // move up to white line and shoot 3 rings into the high goal
        move(24,0.5);
        robot.arm.setArm(Constants.ArmPosition.UP);

        strafe(12,0.55);
        robot.shooter.setShooter(0.630);
        move(40,0.5);
        strafe(-15,0.5);
        shoot();
        shoot();
        shoot();
        robot.shooter.setShooter(0);
        turn(178);

        // move to drop off wobble goal in correct location and move back to the white line
        switch(stack) {
            case NONE:
                strafe(-8,0.5);
                placeGoal();
                strafe(24,0.5);
                move(-12,0.5);
                break;
            case SINGLE:
                move(-22,0.5);
                strafe(12,0.5);
                placeGoal();
                move(6,0.5);
                break;
            case QUAD:
                move(-46,0.5);
                strafe(-6,0.5);
                placeGoal();
                move(30,0.5);
        }

        robot.camera.stopTargetingCamera();
        telemetry.addData("Status", "Finished");
        telemetry.update();
    }
}