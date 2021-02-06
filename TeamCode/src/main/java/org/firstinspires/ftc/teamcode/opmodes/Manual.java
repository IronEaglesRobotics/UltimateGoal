package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.opencv.Detection;
import org.firstinspires.ftc.teamcode.robot.Robot;

import static org.firstinspires.ftc.teamcode.Constants.AUTO_AIM_OFFSET_X;
import static org.firstinspires.ftc.teamcode.Constants.POWERSHOT_SHOOTER_POWER;
import static org.firstinspires.ftc.teamcode.Constants.SHOOTER_POWER;
import static org.firstinspires.ftc.teamcode.Constants.WHEEL_SPEED;
import static org.firstinspires.ftc.teamcode.Constants.WHEEL_TURBO_SPEED;

// Main Driver Program
@TeleOp(name = "Manual")
public class Manual extends OpMode {
    private Robot robot;
    private int shooterPower;
    private int powershotShooterPower;

    private Detection red;
    private Detection blue;
    private Detection powershot;
    private double x;
    private double y;
    private double z;

    private double finishTime;
    private boolean checkPusher;
    private boolean zig;

    private boolean clawPressed;
    private boolean pusherPressed;
    private boolean dpadUpPressed;
    private boolean dpadDownPressed;
    private boolean dpadLeftPressed;
    private boolean dpadRightPressed;

    // Init
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing Robot");
        telemetry.update();
        robot = new Robot(hardwareMap);
        robot.arm.resetEncoder();
        robot.camera.initTargetingCamera();
    }

    // Wait for the first frame after the init button was pressed
    @Override
    public void init_loop() {
        if (robot.camera.getFrameCount() > 0) {
            telemetry.addData("Status", "Initialized");
            telemetry.update();
        }
    }

    // Main loop
    @Override
    public void loop() {
        // driver 1
        // detection and movement constants
        red = robot.camera.getRed();
        blue = robot.camera.getBlue();
        powershot = robot.camera.getPowershots().getLeftMost();
        x = 0;
        y = 0;
        z = 0;
        // driver base control (left bumper activates 'turbo mode')
        if (gamepad1.left_bumper) {
            x = gamepad1.left_stick_x * WHEEL_TURBO_SPEED;
            y = gamepad1.left_stick_y * WHEEL_TURBO_SPEED;
            z = gamepad1.right_stick_x * WHEEL_TURBO_SPEED;
        } else {
            x = gamepad1.left_stick_x * WHEEL_SPEED;
            y = gamepad1.left_stick_y * WHEEL_SPEED;
            z = gamepad1.right_stick_x * WHEEL_SPEED;
        }
        // auto aim at goal
        if (gamepad1.dpad_right) {
            double gx = red.getCenter().x+AUTO_AIM_OFFSET_X;
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
            double px = powershot.getCenter().x+AUTO_AIM_OFFSET_X;
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
        robot.drive.setInput(x, y, z);

        // driver 2
        // move arm up and down
        if (gamepad2.y && gamepad2.dpad_up && !dpadUpPressed) {
            robot.arm.setArm(Constants.ArmPosition.UP);
        } else if (gamepad2.y && gamepad2.dpad_down && !dpadDownPressed) {
            robot.arm.setArm(Constants.ArmPosition.DOWN);
        }
        // open and close claw
        if (gamepad2.b && !clawPressed) {
            robot.arm.setClaw(robot.arm.getClaw() == Constants.ServoPosition.OPEN
                    ? Constants.ServoPosition.CLOSED
                    : Constants.ServoPosition.OPEN);
        }
        // intake
        if (gamepad2.left_bumper) {
            robot.intake.setIntake(-gamepad2.left_trigger*1.0*0.75);
        } else {
            robot.intake.setIntake(gamepad2.left_trigger*1.0*0.75);
        }
        // move pusher in and out
        if (!pusherPressed && gamepad2.a) {
            robot.shooter.setPusher(Constants.ServoPosition.CLOSED);
            finishTime = getRuntime() + 0.4;
            checkPusher = true;
            zig = true;
        }
        pusherPressed = gamepad2.a;
        if (checkPusher && getRuntime() > finishTime) {
            if (zig) {
                robot.shooter.setPusher(Constants.ServoPosition.OPEN);
                finishTime += 0.4;
                zig = false;
            } else {
                zig = true;
                checkPusher = false;
                pusherPressed = false;
            }
        }
        // run the shooter at goal or powershot speed
        if (gamepad2.x) {
            robot.shooter.setShooter(gamepad2.right_trigger*POWERSHOT_SHOOTER_POWER);
        } else {
            robot.shooter.setShooter(gamepad2.right_trigger*SHOOTER_POWER);
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

        // update all of the button press variables (except the pusher one, because of the way it is programmed)
        clawPressed = gamepad2.b;
        dpadUpPressed = gamepad2.dpad_up;
        dpadDownPressed = gamepad2.dpad_down;
        dpadLeftPressed = gamepad2.dpad_left;
        dpadRightPressed = gamepad2.dpad_right;

        // show telemetry
        telemetry.addData("Status", robot.getTelemetry());
        telemetry.update();
    }

    // Stop function called after TeleOp is finished
    @Override
    public void stop() {
        robot.camera.stopTargetingCamera();
    }
}
