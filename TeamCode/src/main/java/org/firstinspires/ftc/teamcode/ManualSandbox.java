package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*Terms:
 * - ms ... Milisecond(s).
 * - Telemetry ... Recording and or transmission of the readings of an instrument. Basically: enviornment sensing.
 * - TFOD ... TensorFlowObjectDetector. Invokes TensorFlow's object detection API, assumably. Read more: https://github.com/tensorflow/models/tree/master/research/object_detection.
 */

// manual driver control
@TeleOp(name = "ManualSandbox")
public class ManualSandbox extends OpMode {
    public int msStuckDetectInit = 15000;

    private Robot robot;
    private boolean clawPressed;
    private boolean pusherPressed;
    private boolean dpad2UpPressed;
    private boolean dpad2DownPressed;
    private boolean dpad2LeftPressed;
    private boolean dpad2RightPressed;
    private double finishTime;
    private boolean checkPusher;
    private boolean zig;
    private double powershotShooterPower = 0.57; // secondary speed for the shooter wheel
    private double shooterPower = 0.62; // primary speed for the shooter wheel

    private Detection red;
    private Detection blue;
    private Detection powerShot;
    private double x;
    private double y;
    private double z;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing Robot");
        telemetry.update();
        robot = new Robot(hardwareMap);
        robot.arm.resetEncoder();
        robot.camera.initTargetingCamera();
    }

    @Override
    public void init_loop() {
        if (robot.camera.getFrameCount() > 0) {
            telemetry.addData("Status", "Initialized");
            telemetry.update();
        }
    }

    @Override
    public void loop() {
        // driver 1
        red = robot.camera.getRed();
        blue = robot.camera.getBlue();
        powerShot = robot.camera.getPowerShot().getLeftMost();

        x = 0;
        y = 0;
        z = 0;

        // normal movements
        if (gamepad1.left_bumper) {
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            z = gamepad1.right_stick_x;
        } else {
            x = gamepad1.left_stick_x*0.7;
            y = -gamepad1.left_stick_y*0.7;
            z = gamepad1.right_stick_x*0.7;
        }

        // auto aim at goal
        if (gamepad1.dpad_right) {
            // Aim for goal
            double gx = red.getCenter().x+10;
            if (Math.abs(gx) < 50) {
                double zMaxSpeed = 0.7;
                double zErr = Math.abs(gx);
                double zSpeed = (zErr / 50) * zMaxSpeed;
                if (zErr <= 1) {
                    z = 0;
                } else if (zErr > 1) {
                    z = Math.copySign(zSpeed, gx);
                }
            }
        }

        // auto aim powershots
        if (gamepad1.dpad_left) {
            // Aim for goal
            double px = powerShot.getCenter().x+10;
            if (Math.abs(px) < 50) {
                double zMaxSpeed = 0.7;
                double zErr = Math.abs(px);
                double zSpeed = (zErr / 50) * zMaxSpeed;
                if (zErr <= 1) {
                    z = 0;
                } else if (zErr > 1) {
                    z = Math.copySign(zSpeed, px);
                }
            }
        }
        robot.drive.setInput(x,y,z);

        // driver 2

        // arm up and down
        if (gamepad2.y && gamepad2.dpad_up && !dpad2UpPressed) {
            robot.arm.setArm(false);
        } else if (gamepad2.y && gamepad2.dpad_down && !dpad2DownPressed) {
            robot.arm.setArm(true);
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
        if (gamepad2.dpad_up && !dpad2UpPressed) {
            powershotShooterPower += 0.01;
        } if (gamepad2.dpad_down && !dpad2DownPressed) {
            powershotShooterPower -= 0.01;
        } if (gamepad2.dpad_left && !dpad2LeftPressed) {
            shooterPower += 0.01;
        } if (gamepad2.dpad_right && !dpad2RightPressed) {
            shooterPower -= 0.01;
        }
        dpad2UpPressed = gamepad2.dpad_up;
        dpad2DownPressed = gamepad2.dpad_down;
        dpad2LeftPressed = gamepad2.dpad_left;
        dpad2RightPressed = gamepad2.dpad_right;

        // show telemetry
        telemetry.addData("Status", robot.getTelemetry());
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.camera.stopTargetingCamera();
    }
}