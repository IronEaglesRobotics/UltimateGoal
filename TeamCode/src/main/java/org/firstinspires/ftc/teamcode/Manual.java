package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static android.os.SystemClock.sleep;

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
    private boolean clawPressed;
    private boolean pusherPressed;
    private double finishTime;
    private boolean checkPusher;
    private boolean zig;

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
        if (gamepad1.right_bumper) {
            robot.drive.setInput(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        } else {
            robot.drive.setInput(gamepad1.left_stick_x*0.7, -gamepad1.left_stick_y*0.7, gamepad1.right_stick_x*0.7);
        }

        // driver 2
        if (!pusherPressed && gamepad2.a) {
            robot.shooter.setPusher(true);//in
            finishTime = getRuntime() + 0.4;
            checkPusher = true;
            zig = true;
        }
        pusherPressed = gamepad2.a;
        if (checkPusher && getRuntime() > finishTime) {
            if (zig) {
                robot.shooter.setPusher(false);
                finishTime += 0.4; // reset time to move arm back out
                zig = false;
            } else {
                zig = true;
                checkPusher = false;
                pusherPressed = false;
            }
        }

        // arm
        robot.arm.setArm(-gamepad2.right_stick_y*0.25);
        if (gamepad2.b && !clawPressed) {
            robot.arm.setClaw(!robot.arm.getClaw());
        }
        clawPressed = gamepad2.b;

        // intake
        if (gamepad2.left_bumper) {
            robot.intake.setIntake(-gamepad2.left_trigger*1.0*0.75);
        } else {
            robot.intake.setIntake(gamepad2.left_trigger*1.0*0.75);
        }

        // shooter
        if (gamepad2.y) {
            robot.shooter.setShooter(-gamepad2.right_trigger*1.0*0.7);
        } else {
            robot.shooter.setShooter(gamepad2.right_trigger*1.0*0.7);
        }

        // show telemetry
        telemetry.addData("Status", robot.getTelemetry());
        telemetry.update();
    }
}