package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/*Terms:
 * - ms ... Milisecond(s).
 * - Telemetry ... Recording and or transmission of the readings of an instrument. Basically: enviornment sensing.
 * - TFOD ... TensorFlowObjectDetector. Invokes TensorFlow's object detection API, assumably. Read more: https://github.com/tensorflow/models/tree/master/research/object_detection.
 */

//Where the TeleOp (driver controlled) code is executed.
@TeleOp(name = "WheelTest")
public class WheelTest extends OpMode {
    public int msStuckDetectInit = 15000;

    private DcMotor wheel;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        wheel = hardwareMap.get(DcMotor.class, "wheel");
        wheel.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        wheel.setPower(gamepad1.right_trigger);

        // show telemetry
//        telemetry.addData("Status", robot.getTelemetry());
        telemetry.addData("Wheel Power", wheel.getPower());
        telemetry.update();
    }
}
