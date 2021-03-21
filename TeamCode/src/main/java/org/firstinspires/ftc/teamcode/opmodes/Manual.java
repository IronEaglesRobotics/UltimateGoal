package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.opencv.Detection;
import org.firstinspires.ftc.teamcode.robot.Robot;

import static org.firstinspires.ftc.teamcode.Constants.ARM_DOWN_POS;
import static org.firstinspires.ftc.teamcode.Constants.ARM_UP_POS;
import static org.firstinspires.ftc.teamcode.Constants.AUTO_AIM_OFFSET_X;
import static org.firstinspires.ftc.teamcode.Constants.CLAW_WAIT;
import static org.firstinspires.ftc.teamcode.Constants.POWERSHOT_SHOOTER_POWER;
import static org.firstinspires.ftc.teamcode.Constants.SHOOTER_POWER;
import static org.firstinspires.ftc.teamcode.Constants.WHEEL_SLOW_SPEED;
import static org.firstinspires.ftc.teamcode.Constants.WHEEL_SPEED;
import static org.firstinspires.ftc.teamcode.Constants.WHEEL_TURBO_SPEED;

// Main Driver Program
@TeleOp(name = "TeleOp", group = "Competition")
public class Manual extends OpMode {
    private Robot robot;

    // detection values
    private Detection red;
    private Detection blue;
    private Detection powershot;
    private boolean aimedAtPowershots;
    private boolean aimedAtGoal;

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
        robot.shooter.setPusher(Constants.ServoPosition.CLOSED);
        robot.arm.setClaw(Constants.ServoPosition.CLOSED);
        armPosition = ARM_UP_POS;
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
        x = gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;
        z = gamepad1.right_stick_x;
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

        powershotPowerPressed = gamepad2.right_bumper;
        shooterPower = gamepad2.right_trigger;
        pusherPressed = gamepad2.a;

        // ------------------------- driver 1 ------------------------- //

        // base control (left trigger adds speed for turbo mode, right trigger removes speed for slow mode)
        x += Math.copySign(turbo * (WHEEL_TURBO_SPEED - WHEEL_SPEED), x) - Math.copySign(slow * (WHEEL_SPEED - WHEEL_SLOW_SPEED), x);
        y += Math.copySign(turbo * (WHEEL_TURBO_SPEED - WHEEL_SPEED), y) - Math.copySign(slow * (WHEEL_SPEED - WHEEL_SLOW_SPEED), y);
        z += Math.copySign(turbo * (WHEEL_TURBO_SPEED - WHEEL_SPEED), z) - Math.copySign(slow * (WHEEL_SPEED - WHEEL_SLOW_SPEED), z);

        // ------------------------- driver 2 ------------------------- //

        // auto aim at powershots
        if ((autoaimPowershots) && powershot.isValid()) {
            double px = powershot.getCenter().x+AUTO_AIM_OFFSET_X;
            if (Math.abs(px) < 50) {
                double zMaxSpeed = 0.7;
                double zErr = Math.abs(px);
                double zSpeed = (zErr / 50) * zMaxSpeed;
                if (zErr <= 1) {
                    z = 0;
                    aimedAtPowershots = true;
                } else if (zErr > 1) {
                    z = Math.copySign(zSpeed, px);
                    aimedAtPowershots = false;
                }
            }
        } else {
            aimedAtPowershots = false;
        }
        // auto aim at goal
        if ((autoaimGoal) && red.isValid()) {
            double gx = red.getCenter().x+AUTO_AIM_OFFSET_X;
            if (Math.abs(gx) < 50) {
                double zMaxSpeed = 0.7;
                double zErr = Math.abs(gx);
                double zSpeed = (zErr / 50) * zMaxSpeed;
                if (zErr <= 1) {
                    z = 0;
                    aimedAtGoal = true;
                } else if (zErr > 1) {
                    z = Math.copySign(zSpeed, gx);
                    aimedAtGoal = false;
                }
            }
        } else {
            aimedAtGoal = false;
        }
        robot.drive.setWeightedDrivePower(new Pose2d(x, y, -z));
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
            robot.intake.setIntake(-intakePower);
        } else {
            robot.intake.setIntake(intakePower);
        }

        // shooter
        if (powershotPowerPressed && !powershotPowerPressedPrev) {
            inPowerShotShooterMode = !inPowerShotShooterMode;
        }
        if (inPowerShotShooterMode) {
            robot.shooter.setShooter(shooterPower * POWERSHOT_SHOOTER_POWER);
        } else {
            robot.shooter.setShooter(shooterPower * SHOOTER_POWER);
        }

        // move pusher in and out
        if (pusherPressed && !pusherPressedPrev) {
            robot.shooter.setPusher(Constants.ServoPosition.CLOSED);
            finishTime = getRuntime() + CLAW_WAIT;
            checkPusher = true;
            zig = true;
        }
        if (checkPusher && getRuntime() > finishTime) {
            if (zig) {
                robot.shooter.setPusher(Constants.ServoPosition.OPEN);
                finishTime += CLAW_WAIT;
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
