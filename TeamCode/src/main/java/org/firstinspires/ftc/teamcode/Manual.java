package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*Terms:
* - ms ... Milisecond(s).
* - Telemetry ... Recording and or transmission of the readings of an instrument. Basically: enviornment sensing.
* - TFOD ... TensorFlowObjectDetector. Invokes TensorFlow's object detection API, assumably. Read more: https://github.com/tensorflow/models/tree/master/research/object_detection.
*/

// manual driver control
@TeleOp(name = "Manual")
public class Manual extends OpMode {
    public int msStuckDetectInit = 15000;

    private Robot robot;
//    private boolean clawPressed;
    private boolean pusherPressed;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
//        robot.setTfodZoom(3);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // driver 1
        robot.drive.setInput(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

//        // driver 2
//        if (gamepad2.right_bumper) {
//            robot.arm.setArm(0.5);
//        } else if (gamepad2.left_bumper) {
//            robot.arm.setArm(-0.5);
//        } else {
//            robot.arm.setArm(0);
//        }
//        if (gamepad2.b && !clawPressed) {
//            robot.arm.setClaw(!robot.arm.getClaw());
//        }
//        clawPressed = gamepad2.b;

        robot.intake.setIntake(-gamepad2.left_stick_y*1.0*0.75);
        robot.shooter.setShooter(gamepad2.right_trigger*1.0*0.7);
        if (gamepad2.a && !pusherPressed) {
            robot.shooter.setPusher(!robot.shooter.getPusher());
        }
        pusherPressed = gamepad2.a;

        // show telemetry
        telemetry.addData("Status", robot.getTelemetry());
        telemetry.update();
    }
}
