package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.opmode.util.controller.Controller;
import org.firstinspires.ftc.teamcode.util.enums.Alliance;
import org.firstinspires.ftc.teamcode.vision.Detection;

import static org.firstinspires.ftc.teamcode.util.Constants.ARM_DEFAULT_POS;
import static org.firstinspires.ftc.teamcode.util.Constants.ARM_DOWN_POS;
import static org.firstinspires.ftc.teamcode.util.Constants.ARM_UP_POS;
import static org.firstinspires.ftc.teamcode.util.Constants.AUTO_AIM_MIN;
import static org.firstinspires.ftc.teamcode.util.Constants.AUTO_AIM_P;
import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE_SPEED;
import static org.firstinspires.ftc.teamcode.util.Constants.PUSHER_DELAY;
import static org.firstinspires.ftc.teamcode.util.Constants.SHOOTER_AUTO_AIM_OFFSET_X;
import static org.firstinspires.ftc.teamcode.util.Constants.SHOOTER_GOAL_POWER;
import static org.firstinspires.ftc.teamcode.util.Constants.SHOOTER_POWERSHOT_POWER;
import static org.firstinspires.ftc.teamcode.util.Constants.WHEEL_SPEED;
import static org.firstinspires.ftc.teamcode.util.enums.Position.CLOSED;
import static org.firstinspires.ftc.teamcode.util.enums.Position.OPEN;
import static org.firstinspires.ftc.teamcode.util.enums.Position.UP;

// Abstract Driver Program
@Disabled
@TeleOp(name = "Red Solo TeleOp", group = "Development")
public class RedSoloTele extends OpMode {
    public Alliance alliance;
    private Robot robot;

    private Controller driver1;

    private int armPosition;

    private double finishTime;
    private boolean checkPusher;
    private boolean zig;

    public void setAlliance() {
        this.alliance = Alliance.RED;
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
        robot.intake.setShield(UP);

        // set current position of the robot
        robot.drive.setPoseEstimate(PoseStorage.currentPose);

        // set gamepads
        driver1 = new Controller(gamepad1);

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

        // base control
        double x = -driver1.getLeftStick().getY() * WHEEL_SPEED;
        double y = -driver1.getLeftStick().getX() * WHEEL_SPEED;
        double z = -driver1.getRightStick().getX() * WHEEL_SPEED;

        // auto aim
        Detection red = robot.camera.getRed();
        Detection blue = robot.camera.getBlue();
        Detection redPowershot = robot.camera.getRedPowershots().getLeftMost();
        Detection bluePowershot = robot.camera.getBluePowershots().getLeftMost();
        if (driver1.getX().isPressed()) {
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
        if (driver1.getY().isPressed()) {
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
        if (driver1.getDUp().isJustPressed()) {
            armPosition = ARM_UP_POS;
        } else if (driver1.getDDown().isJustPressed()) {
            armPosition = ARM_DOWN_POS;
        }
        robot.arm.setArm(armPosition);

        // claw
        if (driver1.getB().isJustPressed()) {
            robot.arm.setClaw(robot.arm.getClaw() == OPEN ? CLOSED : OPEN);
        }

        // intake
        if (driver1.getLeftBumper().isPressed()) {
            robot.intake.setIntake(-driver1.getLeftTrigger().getValue() * INTAKE_SPEED);
        } else {
            robot.intake.setIntake(driver1.getLeftTrigger().getValue() * INTAKE_SPEED);
        }

        // shooter
        if (driver1.getRightBumper().isPressed()) {
            robot.shooter.setShooter(driver1.getRightTrigger().getValue() * SHOOTER_POWERSHOT_POWER);
        } else {
            robot.shooter.setShooter(driver1.getRightTrigger().getValue() * SHOOTER_GOAL_POWER);
        }

        // pusher
        if (driver1.getA().isJustPressed()) {
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
                driver1.getA().update(false);
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
