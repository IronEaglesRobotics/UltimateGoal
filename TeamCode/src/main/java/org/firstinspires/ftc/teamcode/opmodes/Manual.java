package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.opencv.Detection;
import org.firstinspires.ftc.teamcode.robot.Robot;

import static org.firstinspires.ftc.teamcode.Constants.ARM_DEFAULT_POS;
import static org.firstinspires.ftc.teamcode.Constants.ARM_DOWN_POS;
import static org.firstinspires.ftc.teamcode.Constants.ARM_UP_POS;
import static org.firstinspires.ftc.teamcode.Constants.INTAKE_SHIELD_UP;
import static org.firstinspires.ftc.teamcode.Constants.INTAKE_SPEED;
import static org.firstinspires.ftc.teamcode.Constants.PUSHER_DELAY;
import static org.firstinspires.ftc.teamcode.Constants.SHOOTER_AUTO_AIM_OFFSET_X;
import static org.firstinspires.ftc.teamcode.Constants.SHOOTER_GOAL_POWER;
import static org.firstinspires.ftc.teamcode.Constants.SHOOTER_POWERSHOT_POWER;
import static org.firstinspires.ftc.teamcode.Constants.WHEEL_SLOW_SPEED;
import static org.firstinspires.ftc.teamcode.Constants.WHEEL_SPEED;
import static org.firstinspires.ftc.teamcode.Constants.WHEEL_TURBO_SPEED;

// Main Driver Program
@Config
@TeleOp(name = "TeleOp", group = "Competition")
public class Manual extends OpMode {
    // config values
    public static double AUTO_AIM_P = 0.8;
    public static double AUTO_AIM_MIN = 0.15;
    public static double SHIELD_SPEED = 0.02;

    private Robot robot;

    // detection values
    private Detection red;
    private Detection blue;
    private Detection powershot;

    // button presses for driver 1
    private double x;
    private double y;
    private double z;
    private double turbo;
    private double slow;

    // button presses for driver 2
    private boolean autoaimPowershots;
    private boolean autoaimGoal;

    private boolean armUpPressedPrev;
    private boolean armUpPressed;
    private boolean armDownPressedPrev;
    private boolean armDownPressed;
    private int armPosition;
    private double armManual;
    private boolean clawPressedPrev;
    private boolean clawPressed;

    private boolean intakeReversePressed;
    private double intakePower;
    private boolean shieldPressedPrev;
    private boolean shieldPressed;
    public double shieldPosition;
    public double shieldManual;

    private boolean powershotPowerPressedPrev;
    private boolean powershotPowerPressed;
    private boolean inPowerShotShooterMode;
    private double shooterPower;
    private boolean pusherPressedPrev;
    private boolean pusherPressed;
    private double finishTime;
    private boolean checkPusher;
    private boolean zig;

    // Init
    @Override
    public void init() {
        telemetry.addLine("Initializing Robot...");
        telemetry.update();

        robot = new Robot(hardwareMap);
        robot.camera.initTargetingCamera();
        robot.arm.resetEncoder();
        robot.arm.setClaw(Constants.ServoPosition.CLOSED);
        armPosition = ARM_DEFAULT_POS;
        robot.shooter.setPusher(Constants.ServoPosition.OPEN);
        robot.intake.setShield(Constants.ServoPosition.CLOSED);
        shieldPosition = INTAKE_SHIELD_UP;

        // set current position of the robot
        robot.drive.setPoseEstimate(PoseStorage.currentPose);
    }

    // Wait for the first frame after the init button was pressed
    @Override
    public void init_loop() {
        if (robot.camera.getFrameCount() > 0) {
            telemetry.addLine("Initialized");
            telemetry.update();
        }
    }

    // Main loop
    @Override
    public void loop() {
        // update detections
        red = robot.camera.getRed();
        blue = robot.camera.getBlue();
        powershot = robot.camera.getPowershots().getLeftMost();

        // update gamepad presses for driver 1
        x = -gamepad1.left_stick_y;
        y = -gamepad1.left_stick_x;
        z = -gamepad1.right_stick_x;
        turbo = gamepad1.left_trigger;
        slow = gamepad1.right_trigger;

        // update gamepad presses for driver 2
        armUpPressed = gamepad2.dpad_up;
        armDownPressed = gamepad2.dpad_down;
        armManual = -gamepad2.right_stick_y;
        clawPressed = gamepad2.b;
        autoaimPowershots = gamepad2.x;
        autoaimGoal = gamepad2.y;

        intakeReversePressed = gamepad2.left_bumper;
        intakePower = gamepad2.left_trigger;
        shieldPressed = gamepad2.left_stick_button;
        shieldManual = gamepad2.left_stick_y;

        powershotPowerPressed = gamepad2.right_bumper;
        shooterPower = gamepad2.right_trigger;
        pusherPressed = gamepad2.a;

        // ------------------------- driver 1 ------------------------- //

        // base control (left trigger adds speed for turbo mode, right trigger removes speed for slow mode)
        if (Math.abs(x) >= 0.1) {
            x += Math.copySign(turbo * (WHEEL_TURBO_SPEED - WHEEL_SPEED), x);
            x += Math.copySign(Math.min(slow * (WHEEL_SPEED - WHEEL_SLOW_SPEED), Math.abs(x)), x);
        }
        if (Math.abs(y) >= 0.1) {
            y += Math.copySign(turbo * (WHEEL_TURBO_SPEED - WHEEL_SPEED), y);
            y += Math.copySign(Math.min(slow * (WHEEL_SPEED - WHEEL_SLOW_SPEED), Math.abs(x)), y);
        }
        if (Math.abs(z) >= 0.1) {
            z += Math.copySign(turbo * (WHEEL_TURBO_SPEED - WHEEL_SPEED), z);
            z += Math.copySign(Math.min(slow * (WHEEL_SPEED - WHEEL_SLOW_SPEED), Math.abs(x)), z);
        }

        // ------------------------- driver 2 ------------------------- //

        // auto aim
        if (autoaimPowershots) {
            double targetPos = powershot.getCenter().x+ SHOOTER_AUTO_AIM_OFFSET_X;
            if (Math.abs(targetPos) < 0.5) {
                z = 0;
            } else {
                z = Math.copySign(Math.max(Math.abs((targetPos / 50) * AUTO_AIM_P), AUTO_AIM_MIN), -targetPos);
            }
        }
        if (autoaimGoal) {
            double targetPos = red.getCenter().x+ SHOOTER_AUTO_AIM_OFFSET_X;
            if (Math.abs(targetPos) < 0.5) {
                z = 0;
            } else {
                z = Math.copySign(Math.max(Math.abs((targetPos / 50) * AUTO_AIM_P), AUTO_AIM_MIN), -targetPos);
            }
        }
        robot.drive.setWeightedDrivePower(new Pose2d(x, y, z));
        robot.drive.update();

        // move arm up and down
        if (armUpPressed && !armUpPressedPrev) {
            armPosition = ARM_UP_POS;
        } else if (armDownPressed && !armDownPressedPrev) {
            armPosition = ARM_DOWN_POS;
        } else if (Math.abs(armManual) > 0.1) {
            armPosition -= armManual*20;
        }
        robot.arm.setArm(armPosition);

        // open and close claw
        if (clawPressed && !clawPressedPrev) {
            robot.arm.setClaw(robot.arm.getClaw() == Constants.ServoPosition.OPEN
                    ? Constants.ServoPosition.CLOSED
                    : Constants.ServoPosition.OPEN);
        }

        // intake
        if (intakeReversePressed) {
            robot.intake.setIntake(-intakePower * INTAKE_SPEED);
        } else {
            robot.intake.setIntake(intakePower * INTAKE_SPEED);
        }
        if (Math.abs(shieldManual) > 0.1) {
            shieldPosition -= shieldManual * SHIELD_SPEED;
        }
        robot.intake.setShield(shieldPosition);

        // shooter
        if (powershotPowerPressed && !powershotPowerPressedPrev) {
            inPowerShotShooterMode = !inPowerShotShooterMode;
        }
        if (inPowerShotShooterMode) {
            robot.shooter.setShooter(shooterPower * SHOOTER_POWERSHOT_POWER);
        } else {
            robot.shooter.setShooter(shooterPower * SHOOTER_GOAL_POWER);
        }

        // move pusher in and out
        if (pusherPressed && !pusherPressedPrev) {
            robot.shooter.setPusher(Constants.ServoPosition.CLOSED);
            finishTime = getRuntime() + PUSHER_DELAY;
            checkPusher = true;
            zig = true;
        }
        if (checkPusher && getRuntime() > finishTime) {
            if (zig) {
                robot.shooter.setPusher(Constants.ServoPosition.OPEN);
                finishTime += PUSHER_DELAY;
                zig = false;
            } else {
                zig = true;
                checkPusher = false;
                pusherPressed = false;
            }
        }

        // ------------------------------------------------------------ //

        // update previous state variables
        armUpPressedPrev = armUpPressed;
        armDownPressedPrev = armDownPressed;
        clawPressedPrev = clawPressed;
        powershotPowerPressedPrev = powershotPowerPressed;
        pusherPressedPrev = pusherPressed;
        shieldPressedPrev = shieldPressed;

        // show telemetry
        telemetry.addLine(robot.getTelemetry());
        telemetry.update();
    }

    // Stop function called after TeleOp is finished
    @Override
    public void stop() {
        robot.camera.stopTargetingCamera();
    }
}
