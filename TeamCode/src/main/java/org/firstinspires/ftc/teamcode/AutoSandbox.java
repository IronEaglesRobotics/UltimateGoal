package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoSandbox")
public class AutoSandbox extends LinearOpMode {
    private Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (!(isStarted() || isStopRequested())) {
            telemetry.addData("", "waiting to start");
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
        robot.setArmPosition(180, 0.5);
        while(robot.isWobblerBusy()) {
            sleep(1);
        }
        //robot.setClaw(false);

        // stop all motors at the end
        // robot.drive.setPower(0);
    }
}