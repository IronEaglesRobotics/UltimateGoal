package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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
    private boolean dpadUpPressed;
    private boolean dpadDownPressed;
    private boolean dpadLeftPressed;
    private boolean dpadRightPressed;
    private double finishTime;
    private boolean checkPusher;
    private boolean zig;
    private double powershotShooterPower = 0.65; // secondary speed for the shooter wheel
    private double shooterPower = 0.7; // primary speed for the shooter wheel

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.arm.resetEncoder();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // driver 1

        // mecanum drive base
        // the left bumper activates "turbo mode"
        if (gamepad1.left_bumper) {
            robot.drive.setInput(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        } else {
            robot.drive.setInput(gamepad1.left_stick_x*0.7, -gamepad1.left_stick_y*0.7, gamepad1.right_stick_x*0.7);
        }

        // driver 2

        // arm up and down
        if (gamepad2.y && gamepad2.dpad_up && !dpadUpPressed) {
            robot.arm.setArm(true);
        } else if (gamepad2.y && gamepad2.dpad_down && !dpadDownPressed) {
            robot.arm.setArm(false);
        }

        // open and close claw
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

        // move pusher in and out
        if (!pusherPressed && gamepad2.a) {
            robot.shooter.setPusher(true);//in
            finishTime = getRuntime() + 0.4;
            checkPusher = true;
            zig = true;
        }
        pusherPressed = gamepad2.a;
        if (checkPusher && getRuntime() > finishTime) {
            if (zig) {
                robot.shooter.setPusher(false);//out
                finishTime += 0.4; // reset time to move arm back out
                zig = false;
            } else {
                zig = true;
                checkPusher = false;
                pusherPressed = false;
            }
        }

        // run the shooter at primary or secondary speed
        // the thinking is to use a secondary speed for the powershots as opposed to the high goal,
        // but the proper speeds haven't been found yet, more testing needs to be done
        if (gamepad2.x) {
            robot.shooter.setShooter(gamepad2.right_trigger*powershotShooterPower);
        } else {
            robot.shooter.setShooter(gamepad2.right_trigger*shooterPower);
        }

        // dpad is used to change the speed of the primary and secondary speeds for the shooter wheel
        if (gamepad2.dpad_up && !dpadUpPressed) {
            powershotShooterPower += 0.01;
        } if (gamepad2.dpad_down && !dpadDownPressed) {
            powershotShooterPower -= 0.01;
        } if (gamepad2.dpad_left && !dpadLeftPressed) {
            shooterPower += 0.01;
        } if (gamepad2.dpad_right && !dpadRightPressed) {
            shooterPower -= 0.01;
        }
        dpadUpPressed = gamepad2.dpad_up;
        dpadDownPressed = gamepad2.dpad_down;
        dpadLeftPressed = gamepad2.dpad_left;
        dpadRightPressed = gamepad2.dpad_right;

        // show telemetry
        telemetry.addData("Status", robot.getTelemetry());
        telemetry.update();
    }
}
