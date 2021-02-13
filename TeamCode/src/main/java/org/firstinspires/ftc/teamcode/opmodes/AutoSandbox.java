package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.opencv.Detection;
import org.firstinspires.ftc.teamcode.robot.Robot;

import static org.firstinspires.ftc.teamcode.Constants.AUTO_AIM_OFFSET_X;
import static org.firstinspires.ftc.teamcode.Constants.POWERSHOT_SHOOTER_POWER;
import static org.firstinspires.ftc.teamcode.Constants.WHEEL_CIRCUMFERENCE;

// Main Autonomous Program
@Autonomous(name = "Auto Sandbox", group = "Testing", preselectTeleOp = "")
public class AutoSandbox extends LinearOpMode {
    private Robot robot;
    private Constants.StarterStack stack;

    // Move in 2 dimensions
    public void moveXYZ(double x, double y, double power) {
        robot.drive.setTargetPositionRelativeXXX(x, y, power);
        while(robot.drive.isBusy() && opModeIsActive()) {
            sleep(1);
        }
    }

    // Turn
    public void turn(double degrees) {
        final float fudge = 2;
        degrees = Math.abs(degrees);
        robot.imu.resetGyroHeading();
        float current = robot.imu.getGyroHeading360();

        if (degrees > 0) {
            while (current < degrees-fudge || current > 360 - fudge) {
                robot.drive.setInput(0, 0, -(Math.max((degrees-current)/degrees*0.85,0.3)));
                current = robot.imu.getGyroHeading360();

                telemetry.addData("Status", current);
                telemetry.update();
                sleep(1);
            }
        }
        robot.drive.setInput(0,0,0);
    }

    // Absolute Turn
    public void absoluteTurn(double degrees) {
        final float fudge = 2;
        degrees = Math.abs(degrees);
        robot.imu.resetGyroHeadingToInitial();
        float current = 5;

        if (degrees > 0) {
            while (current < degrees-fudge || current > 360 - fudge) {
                robot.drive.setInput(0, 0, -(Math.max((degrees-current)/degrees*0.85,0.3)));
                current = robot.imu.getGyroHeading360();

                telemetry.addData("Status", current);
                telemetry.update();
                sleep(1);
            }
        }
        robot.drive.setInput(0,0,0);
    }

    // Shoot a ring
    public void shootPowershots() {
        int ringsFired = 0;
        double timeout = getRuntime();
        Detection powershot;
        double z = 0;
        boolean aimedAtPowershots = false;

        while (ringsFired < 3 && getRuntime() < timeout + 15000) {
            powershot = robot.camera.getPowershots().getLeftMost();
            if (powershot.isValid()) {
                double px = powershot.getCenter().x+AUTO_AIM_OFFSET_X;
                if (Math.abs(px) < 50) {
                    double zMaxSpeed = 0.7;
                    double zErr = Math.abs(px);
                    double zSpeed = (zErr / 50) * zMaxSpeed;
                    if (zErr <= 0.5) {
                        z = 0;
                        aimedAtPowershots = true;
                    } else {
                        z = Math.copySign(zSpeed, px);
                        aimedAtPowershots = false;
                    }
                }
            } else {
                aimedAtPowershots = false;
            }
            robot.drive.setInput(0, 0, z);

            if (aimedAtPowershots) {
                robot.shooter.setPusher(Constants.ServoPosition.CLOSED);
                sleep(850);
                robot.shooter.setPusher(Constants.ServoPosition.OPEN);
                if (ringsFired++ < 2) {
                    sleep(850);
                }
            }
        }
        robot.shooter.setShooter(0);
    }

    // Place down the goal
    public void placeGoal() {
        double startTime = getRuntime();
        robot.arm.setArm(Constants.ArmPosition.DOWN);
        while(robot.arm.isBusy() && opModeIsActive() && getRuntime() < startTime + 2) {
            sleep(1);
        }

        robot.arm.setClaw(Constants.ServoPosition.OPEN);
        sleep(400);

        robot.arm.setArm(Constants.ArmPosition.UP);
        sleep(500);
    }

    // Main method to run all the steps for autonomous
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing Robot");
        telemetry.update();
        robot = new Robot(hardwareMap);
        robot.camera.initWobbleGoalCamera();

        // wait for the first frame to make sure it doesn't try to analyze a non existent frame
        while (robot.camera.getFrameCount() < 1) {
            this.sleep(1);
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // wait for start
        while (!(isStarted() || isStopRequested())) {
            sleep(1);
        }

        // movements for auto
        turn(90);
        sleep(2000);

        turn(90);
        sleep(2000);

        turn(90);
        sleep(2000);

        turn(90);
        sleep(2000);

        turn(180);
        sleep(2000);

        turn(360);

        // move depending on the starting configuration

        robot.camera.stopWobbleGoalCamera();
        telemetry.addData("Status", "Finished");
        telemetry.update();
    }
}