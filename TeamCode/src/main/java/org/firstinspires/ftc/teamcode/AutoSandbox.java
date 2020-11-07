package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "AutoSandbox")
public class AutoSandbox extends LinearOpMode {
    private Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.setClaw(false);

        while (!(isStarted() || isStopRequested())) {
            telemetry.addData("Status", "waiting to start");
            telemetry.update();
            idle();
        }

        // drive to white line
        robot.drive.setForwardTargetPositionRelative(40, 0.5);
        while(robot.drive.isBusy()) {
            sleep(1);
        }

        // move over to square A
        robot.drive.setTargetStrafePositionRelative(10, 0.5);
        while(robot.drive.isBusy()) {
            sleep(1);
        }

        // move arm to release wobble goal
        robot.setArmPosition(180f, 0.5);
        while(robot.isWobblerBusy()) {
            sleep(1);
        }

        robot.setClaw(true);
        sleep(1000);

        telemetry.addData("Status", "finished");
        telemetry.update();
        // stop all motors at the end
        // robot.drive.setPower(0);
    }
}