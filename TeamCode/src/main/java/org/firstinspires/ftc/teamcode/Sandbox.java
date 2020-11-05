package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Sandbox")
public class Sandbox extends OpMode {
    public int msStuckDetectInit = 10000;

    private Robot robot;
    private boolean buttonPressed;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        robot.drive.setInput(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

        if (gamepad1.right_bumper) {
            robot.setArm(Robot.ARM_POWER);
        } else if (gamepad1.left_bumper) {
            robot.setArm(-Robot.ARM_POWER);
        } else {
            robot.setArm(0);
        }

        if (gamepad1.a && !buttonPressed) {
            robot.setClaw(!robot.getClaw());
        }
        buttonPressed = gamepad1.a;

        // show telemetry
        telemetry.addData("", robot.getTelemetry());
        telemetry.update();
    }
}
