package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.opencv.Detection;
import org.firstinspires.ftc.teamcode.robot.Robot;

import static org.firstinspires.ftc.teamcode.Constants.ARM_SPEED;
import static org.firstinspires.ftc.teamcode.Constants.AUTO_AIM_OFFSET_X;
import static org.firstinspires.ftc.teamcode.Constants.INTAKE_MAX_SPEED;
import static org.firstinspires.ftc.teamcode.Constants.POWERSHOT_SHOOTER_POWER;
import static org.firstinspires.ftc.teamcode.Constants.SHOOTER_POWER;
import static org.firstinspires.ftc.teamcode.Constants.WHEEL_SLOW_SPEED;
import static org.firstinspires.ftc.teamcode.Constants.WHEEL_SPEED;
import static org.firstinspires.ftc.teamcode.Constants.WHEEL_TURBO_SPEED;

// Main Driver Program
@TeleOp(name = "Single Driver TeleOp", group = "Testing")
public class ManualSolo extends OpMode {
    private Robot robot;

    // detection values
    private Detection red;
    private Detection blue;
    private Detection powershot;
    private boolean aimedAtPowershots;
    private boolean aimedAtGoal;

    // button presses for the driver
    private double x;
    private double y;
    private double z;
    private double turbo;

    private boolean autoaimPowershots;
    private boolean autoaimGoal;

    private boolean armUpPressedPrev;
    private boolean armDownPressedPrev;
    private boolean armUpPressed;
    private boolean armDownPressed;
    private boolean armResetPressedPrev;
    private boolean armResetPressed;
    private boolean clawPressedPrev;
    private boolean clawPressed;

    private boolean intakeReversePressed;
    private double intakePower;

    private boolean shooterPressedPrev;
    private boolean powershotPressedPrev;
    private boolean shooterPressed;
    private boolean powershotPressed;
    private boolean shooterOn;
    private boolean powershotOn;

    private boolean pusherPressedPrev;
    private double pusherPressed;
    private double finishTime;
    private boolean checkPusher;
    private boolean zig;

    // Init
    @Override
    public void init() {
        telemetry.addLine("Initializing Robot...");
        telemetry.update();
        robot = new Robot(hardwareMap);
        robot.arm.resetEncoder();
        robot.arm.setClaw(Constants.ServoPosition.OPEN);
        robot.shooter.setPusher(Constants.ServoPosition.OPEN);
        robot.camera.initTargetingCamera();
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
        x = gamepad1.left_stick_x*WHEEL_SPEED;
        y = -gamepad1.left_stick_y*WHEEL_SPEED;
        z = gamepad1.right_stick_x*WHEEL_SPEED;

        // update gamepad presses for driver 2
        armUpPressed = gamepad1.dpad_up;
        armDownPressed = gamepad1.dpad_down;
        armResetPressed = gamepad1.dpad_left;
        clawPressed = gamepad1.b;
        autoaimPowershots = gamepad1.left_bumper;
        autoaimGoal = gamepad1.right_bumper;

        intakeReversePressed = gamepad1.a;
        intakePower = gamepad1.left_trigger;
        shooterPressed = gamepad1.y;
        powershotPressed = gamepad1.x;
        pusherPressed = gamepad1.right_trigger;

        // ------------------------- driver 1 ------------------------- //

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
        robot.drive.setInput(x, y, z);

        // move arm up and down
        if (armUpPressed && !armUpPressedPrev) {
            robot.arm.setArm(Constants.ArmPosition.UP);
        } else if (armDownPressed && !armDownPressedPrev) {
            robot.arm.setArm(Constants.ArmPosition.DOWN);
        }
        if (armResetPressed && !armResetPressedPrev) {
            robot.arm.resetEncoder();
        }

        // open and close claw
        if (clawPressed && !clawPressedPrev) {
            robot.arm.setClaw(robot.arm.getClaw() == Constants.ServoPosition.OPEN
                    ? Constants.ServoPosition.CLOSED
                    : Constants.ServoPosition.OPEN);
        }

        // intake
        if (intakeReversePressed) {
            robot.intake.setIntake(-intakePower * INTAKE_MAX_SPEED);
        } else {
            robot.intake.setIntake(intakePower * INTAKE_MAX_SPEED);
        }

        // shooter
        if (shooterPressed && !shooterPressedPrev) {
            shooterOn = !shooterOn;
            powershotOn = false;
        } else if (powershotPressed && !powershotPressedPrev) {
            shooterOn = false;
            powershotOn = !powershotOn;
        }
        if (shooterOn) {
            robot.shooter.setShooter(SHOOTER_POWER);
        } else if (powershotOn) {
            robot.shooter.setShooter(POWERSHOT_SHOOTER_POWER);
        } else {
            robot.shooter.setShooter(0);
        }

        // move pusher in and out
        if (pusherPressed > 0.9 && !pusherPressedPrev) {
            robot.shooter.setPusher(Constants.ServoPosition.CLOSED);
            finishTime = getRuntime() + 0.35;
            checkPusher = true;
            zig = true;
        }
        if (checkPusher && getRuntime() > finishTime) {
            if (zig) {
                robot.shooter.setPusher(Constants.ServoPosition.OPEN);
                finishTime += 0.35;
                zig = false;
            } else {
                zig = true;
                checkPusher = false;
                pusherPressed = 0;
            }
        }

        // ------------------------------------------------------------ //

        // update previous state variables
        armUpPressedPrev = armUpPressed;
        armDownPressedPrev = armDownPressed;
        armResetPressedPrev = armResetPressed;
        clawPressedPrev = clawPressed;
        shooterPressedPrev = shooterPressed;
        powershotPressedPrev = powershotPressed;
        pusherPressedPrev = pusherPressed > 0.9;

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
