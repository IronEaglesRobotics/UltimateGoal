package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoSandbox")
public class AutoSandbox extends LinearOpMode {
    private Robot robot;

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
        robot.drive.setTargetTurnPositionRelative(degrees, power);
        while(robot.drive.isBusy() && opModeIsActive()) {
            sleep(1);
        }
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

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (!(isStarted() || isStopRequested())) {
            telemetry.addData("Status", "Waiting for start");
            telemetry.update();
            idle();
        }

        turn(360, 0.5);
        move(10, 0.5);
        placeGoal();
        turn(-360, 0.5);

        telemetry.addData("Status", "Finished");
        telemetry.update();
    }
}