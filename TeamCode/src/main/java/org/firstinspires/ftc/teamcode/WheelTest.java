package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/*Terms:
 * - ms ... Milisecond(s).
 * - Telemetry ... Recording and or transmission of the readings of an instrument. Basically: enviornment sensing.
 * - TFOD ... TensorFlowObjectDetector. Invokes TensorFlow's object detection API, assumably. Read more: https://github.com/tensorflow/models/tree/master/research/object_detection.
 */

//Where the TeleOp (driver controlled) code is executed.
@TeleOp(name = "WheelTest")
public class WheelTest extends OpMode {
    public int msStuckDetectInit = 15000;

//    private DcMotor wheel;
//    private DcMotor intake;
    private Servo pusher;

    //Claw control.
    private static final double PUSHER_MIN = 0.35; // pusher in the hopper
    private static final double PUSHER_MAX = 0.85; // pusher out of the hopper
    private boolean pusherPressed;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
//        wheel = hardwareMap.get(DcMotor.class, "wheel");
//        wheel.setDirection(DcMotor.Direction.REVERSE);
//        intake = hardwareMap.get(DcMotor.class, "intake");
//        intake.setDirection(DcMotor.Direction.REVERSE);
//        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pusher = hardwareMap.get(Servo.class, "pusher");
        pusher.scaleRange(PUSHER_MIN, PUSHER_MAX);
    }

    //Pusher position setter.
    public void setPusher(boolean open) {
        pusher.setPosition(open ? 0 : 1);
    }

    //Pusher position getter.
    public boolean getPusher() {
        return pusher.getPosition() < 0.4;
    }

    @Override
    public void loop() {
//        if (gamepad1.x) {
//            wheel.setPower(wheel.getPower()-0.01);
//        } if (gamepad1.a) {
//            wheel.setPower(wheel.getPower()-0.01);
//        } if (gamepad1.b) {
//            wheel.setPower(wheel.getPower()+0.01);
//        }
//
//        if (gamepad1.y) {
//            intake.setPower(0.5);
//        }
//        wheel.setPower(gamepad1.right_trigger/1.0*0.7);
//        intake.setPower(gamepad1.left_trigger/1.0*0.75);
//        if (gamepad1.left_trigger > 0.5) {
//            intake.setPower(0.5);
//        } else {
//            intake.setPower(0.0);
//        }
        if (gamepad1.a && !pusherPressed) {
            setPusher(!getPusher());
        }
        pusherPressed = gamepad1.a;


        // show telemetry
//        telemetry.addData("Status", robot.getTelemetry());
//        telemetry.addData("Wheel Power", wheel.getPower());
//        telemetry.addData("Intake Power", intake.getPower());
        telemetry.addData("pusher", getPusher());
        telemetry.update();
    }
}
