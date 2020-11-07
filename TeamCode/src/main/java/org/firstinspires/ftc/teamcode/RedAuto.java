package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "RedAuto")
public class RedAuto extends LinearOpMode {
    private Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);
        robot.setTfodZoom(3);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.setClaw(false);

        while (!(isStarted() || isStopRequested())) {
            telemetry.addData("Status", "waiting to start");
            telemetry.update();
            idle();
        }

        switch(robot.checkStack()) {
            case NONE:
                robot.drive.setForwardTargetPositionRelative(6, 0.5);
                while(robot.drive.isBusy()) {
                    sleep(1);
                }

                robot.drive.setTargetStrafePositionRelative(24, 0.5);
                while(robot.drive.isBusy()) {
                    sleep(1);
                }

                robot.drive.setForwardTargetPositionRelative(48, 0.5);
                while(robot.drive.isBusy()) {
                    sleep(1);
                }

                robot.setArmPosition(120, 0.5);
                while(robot.isWobblerBusy()) {
                    sleep(1);
                }

                robot.setClaw(true);
                sleep(1000);

                robot.drive.setForwardTargetPositionRelative(-6, 0.5);
                while(robot.drive.isBusy()) {
                    sleep(1);
                }
                break;
            case SINGLE:
                robot.drive.setForwardTargetPositionRelative(80, 0.5);
                while(robot.drive.isBusy()) {
                    sleep(1);
                }

                robot.drive.setTargetStrafePositionRelative(-6, 0.5);
                while(robot.drive.isBusy()) {
                    sleep(1);
                }

                robot.setArmPosition(120, 0.5);
                while(robot.isWobblerBusy()) {
                    sleep(1);
                }

                robot.setClaw(true);
                sleep(1000);

                robot.drive.setForwardTargetPositionRelative(-6, 0.5);
                while(robot.drive.isBusy()) {
                    sleep(1);
                }
                break;
            case QUAD:
                robot.drive.setForwardTargetPositionRelative(6, 0.5);
                while(robot.drive.isBusy()) {
                    sleep(1);
                }

                robot.drive.setTargetStrafePositionRelative(24, 0.5);
                while(robot.drive.isBusy()) {
                    sleep(1);
                }

                robot.drive.setForwardTargetPositionRelative(96, 0.5);
                while(robot.drive.isBusy()) {
                    sleep(1);
                }

                robot.setArmPosition(120, 0.5);
                while(robot.isWobblerBusy()) {
                    sleep(1);
                }

                robot.setClaw(true);
                sleep(1000);

                robot.drive.setForwardTargetPositionRelative(-6, 0.5);
                while(robot.drive.isBusy()) {
                    sleep(1);
                }
        }

        telemetry.addData("Status", "finished");
        telemetry.update();
        // stop all motors at the end
        // robot.drive.setPower(0);
    }
}