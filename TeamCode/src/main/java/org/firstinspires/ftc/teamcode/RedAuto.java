package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//Execution code for the CV and whatever else.
@Autonomous(name = "RedAuto")
public class RedAuto extends LinearOpMode {
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

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.arm.setClaw(false);

        while (!(isStarted() || isStopRequested())) {
            telemetry.addData("Status", "waiting to start");
            telemetry.update();
            idle();
        }

        move(6,0.5);
        strafe(10,0.5);

        switch(robot.checkStack()) {
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
                strafe(8, 0.5);
                move(96, 0.5);
                placeGoal();
                move(-48, 0.5);
                strafe(-24, 0.5);
        }

        telemetry.addData("Status", "finished");
        telemetry.update();
    }
}