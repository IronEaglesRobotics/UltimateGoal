package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.Detection.INVALID_POINT;

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
    private boolean dpadUpPressed;
    private boolean dpadDownPressed;
    private boolean dpadLeftPressed;
    private boolean dpadRightPressed;
    private double finishTime;
    private boolean checkPusher;
    private boolean zig;
    private double powershotShooterPower = 0.57; // secondary speed for the shooter wheel
    private double shooterPower = 0.62; // primary speed for the shooter wheel
    private CRServo spinner;

    private Detection red;
    private Detection blue;
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
        spinner = hardwareMap.get(CRServo.class, "spinner");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // driver 1
        red = robot.camera.getRed();
        blue = robot.camera.getBlue();

        x = 0;
        y = 0;
        z = 0;

        x = gamepad1.left_stick_x*0.7;
        y = -gamepad1.left_stick_y*0.7;
        z = gamepad1.right_stick_x*0.7;
        if (gamepad1.right_bumper) {
            // Aim for goal
            double gx = red.getCenter().x;
            if (gx != INVALID_POINT.x) {
                double zMaxSpeed = 0.7;
                double zErr = Math.abs(red.getCenter().x);
                double zSpeed = (zErr / 50) * zMaxSpeed;
                if (zErr <= 1) {
                    z = 0;
                } else if (zErr > 1) {
                    z = Math.copySign(zSpeed, red.getCenter().x);
                }
            }
            telemetry.addData("Red", String.format(Locale.US, "Area: %.1f, Center: (%.1f, %.1f)", red.getArea(), red.getCenter().x, red.getCenter().y));
            telemetry.addData("Blue", String.format(Locale.US, "Area: %.1f, Center: (%.1f, %.1f)", blue.getArea(), blue.getCenter().x, blue.getCenter().y));
        } else if (gamepad1.left_bumper) {
            telemetry.addData("Red", String.format(Locale.US, "Area: %.1f, Center: (%.1f, %.1f)", red.getArea(), red.getCenter().x, red.getCenter().y));
            telemetry.addData("Blue", String.format(Locale.US, "Area: %.1f, Center: (%.1f, %.1f)", blue.getArea(), blue.getCenter().x, blue.getCenter().y));
            // Aim for powershot
            PowerShotDetection powershots = robot.camera.getPowerShots();
            int count = powershots.getCount();
            telemetry.addData("PS Count", powershots.getCount());
            if (count > 0) {
                double px = powershots.get(0).getCenter().x;
                if (px != INVALID_POINT.x) {
                    double zMaxSpeed = 0.7;
                    double zErr = Math.abs(powershots.get(0).getCenter().x);
                    double zSpeed = (zErr / 50) * zMaxSpeed;
                    if (zErr <= 1) {
                        z = 0;
                    } else if (zErr > 1) {
                        z = Math.copySign(zSpeed, red.getCenter().x);
                    }
                }
                telemetry.addData("PS 1 Center", powershots.get(0).getCenter());
                if (count > 1) {
                    telemetry.addData("PS 2 Center", powershots.get(1).getCenter());
                }
                if (count > 2) {
                    telemetry.addData("PS 3 Center", powershots.get(2).getCenter());
                }
            }
        }
        robot.drive.setInput(x,y,z);

        // driver 2

        // arm up and down
        if (gamepad2.y && gamepad2.dpad_up && !dpadUpPressed) {
            robot.arm.setArm(false);
        } else if (gamepad2.y && gamepad2.dpad_down && !dpadDownPressed) {
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

        // test servo
        spinner.setPower(-gamepad2.right_stick_y);

        // show telemetry
        telemetry.addData("Status", robot.getTelemetry());
        telemetry.addData("Spinner Power", spinner.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.camera.stopTargetingCamera();
    }
}