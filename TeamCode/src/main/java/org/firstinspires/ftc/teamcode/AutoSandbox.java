 package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//Sandbox auto program
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
//        robot.arm.setTargetArmPosition(120, 0.5);
//        while(robot.arm.isBusy() && opModeIsActive()) {
//            sleep(1);
//        }
//
//        robot.arm.setClaw(true);
//        sleep(1000);
//
//        move(-2, 0.5);
//
//        robot.arm.setTargetArmPosition(-120, 0.5);
//        while(robot.arm.isBusy() && opModeIsActive()) {
//            sleep(1);
//        }
    }

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (!(isStarted() || isStopRequested())) {
            idle();
        }

        turn(90, 0.5);

        telemetry.addData("Status", "Finished");
        telemetry.update();
    }
}