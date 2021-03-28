package org.firstinspires.ftc.teamcode.opmode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.vision.Detection;
import org.firstinspires.ftc.teamcode.opmode.util.controller.Controller;
import org.firstinspires.ftc.teamcode.util.enums.Alliance;

import static org.firstinspires.ftc.teamcode.util.Constants.ARM_DEFAULT_POS;
import static org.firstinspires.ftc.teamcode.util.Constants.ARM_DOWN_POS;
import static org.firstinspires.ftc.teamcode.util.Constants.ARM_UP_POS;
import static org.firstinspires.ftc.teamcode.util.Constants.AUTO_AIM_MIN;
import static org.firstinspires.ftc.teamcode.util.Constants.AUTO_AIM_P;
import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE_SHIELD_UP;
import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE_SPEED;
import static org.firstinspires.ftc.teamcode.util.Constants.PUSHER_DELAY;
import static org.firstinspires.ftc.teamcode.util.Constants.SHIELD_SPEED;
import static org.firstinspires.ftc.teamcode.util.Constants.SHOOTER_AUTO_AIM_OFFSET_X;
import static org.firstinspires.ftc.teamcode.util.Constants.SHOOTER_GOAL_POWER;
import static org.firstinspires.ftc.teamcode.util.Constants.SHOOTER_POWERSHOT_POWER;
import static org.firstinspires.ftc.teamcode.util.Constants.WHEEL_SLOW_SPEED;
import static org.firstinspires.ftc.teamcode.util.Constants.WHEEL_SPEED;
import static org.firstinspires.ftc.teamcode.util.Constants.WHEEL_TURBO_SPEED;
import static org.firstinspires.ftc.teamcode.util.enums.Position.CLOSED;
import static org.firstinspires.ftc.teamcode.util.enums.Position.OPEN;

// Abstract Driver Program
public abstract class Tele extends OpMode {
    public Alliance alliance;
    private Robot robot;

    private Controller driver1;
    private Controller driver2;

    private int armPosition;
    public double shieldPosition;

    private boolean inPowerShotShooterMode;
    private double finishTime;
    private boolean checkPusher;
    private boolean zig;

    public void setAlliance() {
        this.alliance = Alliance.NEITHER;
    }

    // Init
    @Override
    public void init() {
        telemetry.addLine("Initializing Robot...");
        telemetry.update();

        robot = new Robot(hardwareMap);
        robot.camera.initTargetingCamera();
        robot.arm.resetEncoder();
        robot.arm.setClaw(CLOSED);
        armPosition = ARM_DEFAULT_POS;
        robot.shooter.setPusher(OPEN);
        shieldPosition = INTAKE_SHIELD_UP;
        robot.intake.setShield(shieldPosition);

        // set current position of the robot
        robot.drive.setPoseEstimate(PoseStorage.currentPose);

        // set gamepads
        driver1 = new Controller(gamepad1);
        driver2 = new Controller(gamepad2);

        setAlliance();
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
        driver1.update();
        driver2.update();

        // base control
        double x = driver1.getLeftStick().getX();
        double y = driver1.getLeftStick().getY();
        double z = driver1.getRightStick().getX();
        if (Math.abs(x) >= 0.1) {
            x += Math.copySign(driver1.getLeftTrigger().getValue() * (WHEEL_TURBO_SPEED - WHEEL_SPEED), x);
            x -= Math.copySign(Math.min(driver1.getRightTrigger().getValue() * (WHEEL_SPEED - WHEEL_SLOW_SPEED), Math.abs(x)), x);
        }
        if (Math.abs(y) >= 0.1) {
            y += Math.copySign(driver1.getLeftTrigger().getValue() * (WHEEL_TURBO_SPEED - WHEEL_SPEED), y);
            y -= Math.copySign(Math.min(driver1.getRightTrigger().getValue() * (WHEEL_SPEED - WHEEL_SLOW_SPEED), Math.abs(x)), y);
        }
        if (Math.abs(z) >= 0.1) {
            z += Math.copySign(driver1.getLeftTrigger().getValue() * (WHEEL_TURBO_SPEED - WHEEL_SPEED), z);
            z -= Math.copySign(Math.min(driver1.getRightTrigger().getValue() * (WHEEL_SPEED - WHEEL_SLOW_SPEED), Math.abs(x)), z);
        }

        // auto aim
        Detection red = robot.camera.getRed();
        Detection blue = robot.camera.getBlue();
        Detection redPowershot = robot.camera.getRedPowershots().getLeftMost();
        Detection bluePowershot = robot.camera.getBluePowershots().getLeftMost();
        if (driver2.getX().isPressed()) {
            double targetPos = 0;
            if (alliance == Alliance.RED) {
                targetPos = redPowershot.getCenter().x+ SHOOTER_AUTO_AIM_OFFSET_X;
            } else if (alliance == Alliance.BLUE) {
                targetPos = bluePowershot.getCenter().x+ SHOOTER_AUTO_AIM_OFFSET_X;
            }
            if (Math.abs(targetPos) < 0.5) {
                z = 0;
            } else {
                z = Math.copySign(Math.max(Math.abs((targetPos / 50) * AUTO_AIM_P), AUTO_AIM_MIN), -targetPos);
            }
        }
        if (driver2.getY().isPressed()) {
            double targetPos = 0;
            if (alliance == Alliance.RED) {
                targetPos = red.getCenter().x+ SHOOTER_AUTO_AIM_OFFSET_X;
            } else if (alliance == Alliance.BLUE) {
                targetPos = blue.getCenter().x+ SHOOTER_AUTO_AIM_OFFSET_X;
            }
            if (Math.abs(targetPos) < 0.5) {
                z = 0;
            } else {
                z = Math.copySign(Math.max(Math.abs((targetPos / 50) * AUTO_AIM_P), AUTO_AIM_MIN), -targetPos);
            }
        }
        robot.drive.setWeightedDrivePower(new Pose2d(x, y, z));
        robot.drive.update();

        // save current robot position
        PoseStorage.currentPose = robot.drive.getPoseEstimate();

        // arm
        if (driver2.getDUp().isJustPressed()) {
            armPosition = ARM_UP_POS;
        } else if (driver2.getDDown().isJustPressed()) {
            armPosition = ARM_DOWN_POS;
        } else if (Math.abs(driver2.getLeftStick().getY()) > 0.1) {
            armPosition -= driver2.getLeftStick().getY()*20;
        } else {
            armPosition = robot.arm.getArm();
        }
        robot.arm.setArm(armPosition);

        // claw
        if (driver2.getB().isJustPressed()) {
            robot.arm.setClaw(robot.arm.getClaw() == OPEN ? CLOSED : OPEN);
        }

        // intake
        if (driver2.getLeftBumper().isPressed()) {
            robot.intake.setIntake(-driver2.getLeftTrigger().getValue() * INTAKE_SPEED);
        } else {
            robot.intake.setIntake(driver2.getLeftTrigger().getValue() * INTAKE_SPEED);
        }
        if (Math.abs(driver2.getLeftStick().getY()) > 0.1) {
            shieldPosition -= driver2.getLeftStick().getY() * SHIELD_SPEED;
        }
        robot.intake.setShield(shieldPosition);

        // shooter
        if (driver2.getRightBumper().isJustPressed()) {
            inPowerShotShooterMode = !inPowerShotShooterMode;
        }
        if (inPowerShotShooterMode) {
            robot.shooter.setShooter(driver2.getRightTrigger().getValue() * SHOOTER_POWERSHOT_POWER);
        } else {
            robot.shooter.setShooter(driver2.getRightTrigger().getValue() * SHOOTER_GOAL_POWER);
        }

        // pusher
        if (driver2.getB().isJustPressed()) {
            robot.shooter.setPusher(CLOSED);
            finishTime = getRuntime() + PUSHER_DELAY;
            checkPusher = true;
            zig = true;
        }
        if (checkPusher && getRuntime() > finishTime) {
            if (zig) {
                robot.shooter.setPusher(OPEN);
                finishTime += PUSHER_DELAY;
                zig = false;
            } else {
                zig = true;
                checkPusher = false;
                driver2.getB().update(false);
            }
        }

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
