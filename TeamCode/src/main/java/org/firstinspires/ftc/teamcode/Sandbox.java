package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*Terms:
* - ms ... Milisecond(s).
* - Telemetry ... Recording and or transmission of the readings of an instrument. Basically: enviornment sensing.
* - TFOD ... TensorFlowObjectDetector. Invokes TensorFlow's object detection API, assumably. Read more: https://github.com/tensorflow/models/tree/master/research/object_detection.
*/

//Where the TeleOp (driver controlled) code is executed.
@TeleOp(name = "Sandbox")
public class Sandbox extends OpMode {
    public int msStuckDetectInit = 15000;

    private Robot robot;
    private boolean buttonPressed;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.setTfodZoom(3);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        robot.drive.setInput(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

        if (gamepad1.right_bumper) {
            robot.arm.setArm(0.5);
        } else if (gamepad1.left_bumper) {
            robot.arm.setArm(-0.5);
        } else {
            robot.arm.setArm(0);
        }

        if (gamepad1.a && !buttonPressed) {
            robot.arm.setClaw(!robot.arm.getClaw());
        }

        buttonPressed = gamepad1.a;

        robot.setWheel(gamepad1.right_trigger);

        // show telemetry
        telemetry.addData("Status", robot.getTelemetry());
        telemetry.addData("Wheel Power", robot.getWheelPower());
        telemetry.update();
    }
}
