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
            telemetry.addData("Status", "Waiting for start");
            telemetry.update();
            idle();
        }

        robot.drive.setTargetTurnPositionRelative(360, 0.5);
        while(robot.drive.isBusy()) {
            sleep(1);
        }

        telemetry.addData("Status", "Finished");
        telemetry.update();
    }
}