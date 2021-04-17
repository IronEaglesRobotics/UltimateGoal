package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.opmode.util.controller.Controller;
import org.firstinspires.ftc.teamcode.util.enums.Alliance;
import org.firstinspires.ftc.teamcode.vision.Detection;

import static org.firstinspires.ftc.teamcode.util.Configurables.ARM_DEFAULT_POS;
import static org.firstinspires.ftc.teamcode.util.Configurables.ARM_DOWN_POS;
import static org.firstinspires.ftc.teamcode.util.Configurables.ARM_SPEED;
import static org.firstinspires.ftc.teamcode.util.Configurables.ARM_UP_POS;
import static org.firstinspires.ftc.teamcode.util.Configurables.AUTO_AIM_MIN;
import static org.firstinspires.ftc.teamcode.util.Configurables.AUTO_AIM_P;
import static org.firstinspires.ftc.teamcode.util.Configurables.AUTO_AIM_PMAX;
import static org.firstinspires.ftc.teamcode.util.Configurables.INTAKE_SHIELD_DOWN;
import static org.firstinspires.ftc.teamcode.util.Configurables.INTAKE_SHIELD_UP;
import static org.firstinspires.ftc.teamcode.util.Configurables.INTAKE_SPEED;
import static org.firstinspires.ftc.teamcode.util.Configurables.PUSHER_DELAY;
import static org.firstinspires.ftc.teamcode.util.Configurables.SHIELD_SPEED;
import static org.firstinspires.ftc.teamcode.util.Configurables.SHOOTER_AUTO_AIM_OFFSET_X;
import static org.firstinspires.ftc.teamcode.util.Configurables.SHOOTER_GOAL_POWER;
import static org.firstinspires.ftc.teamcode.util.Configurables.SHOOTER_POWERSHOT_POWER;
import static org.firstinspires.ftc.teamcode.util.enums.Position.CLOSED;
import static org.firstinspires.ftc.teamcode.util.enums.Position.OPEN;

// Driver Program
@TeleOp(name = "Red TeleOp", group = "Competition")
public class RedTele extends OpMode {
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
        double x = -driver1.getLeftStick().getY();
        double y = -driver1.getLeftStick().getX();
        double z = -driver1.getRightStick().getX();

        // auto aim
        Detection red = robot.camera.getRed();
        Detection blue = robot.camera.getBlue();
        Detection redPowershot = robot.camera.getRedPowershots().getLeftMost();
        Detection bluePowershot = robot.camera.getBluePowershots().getLeftMost();
        // powershots
        if (driver2.getX().isPressed()) {
            inPowerShotShooterMode = true;
            double targetPos = 0;
            if (alliance == Alliance.RED) {
                targetPos = redPowershot.getCenter().x + SHOOTER_AUTO_AIM_OFFSET_X;
            } else if (alliance == Alliance.BLUE) {
                targetPos = bluePowershot.getCenter().x + SHOOTER_AUTO_AIM_OFFSET_X;
            }
            if (Math.abs(targetPos) < 0.5) {
                z = 0;
            } else {
                z = Math.copySign(Math.max(Math.abs((targetPos / AUTO_AIM_PMAX) * AUTO_AIM_P), AUTO_AIM_MIN), -targetPos);
            }
        }
        // goal
        if (driver2.getY().isPressed()) {
            inPowerShotShooterMode = false;
            double targetPos = 0;
            if (alliance == Alliance.RED) {
                targetPos = red.getCenter().x + SHOOTER_AUTO_AIM_OFFSET_X;
            } else if (alliance == Alliance.BLUE) {
                targetPos = blue.getCenter().x + SHOOTER_AUTO_AIM_OFFSET_X;
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
        } else if (Math.abs(driver2.getRightStick().getY()) > 0.1) {
            armPosition += driver2.getRightStick().getY()*ARM_SPEED;
        } else {
            armPosition = robot.arm.getArm();
        }
//        if (armPosition < ARM_UP_POS && armPosition < ARM_DOWN_POS) {
//            armPosition = Math.min(ARM_UP_POS, ARM_DOWN_POS);
//        } else if (armPosition > ARM_UP_POS && armPosition > ARM_DOWN_POS) {
//            armPosition = Math.max(ARM_UP_POS, ARM_DOWN_POS);
//        }
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

        // shield
        if (Math.abs(driver2.getLeftStick().getY()) > 0.1) {
            shieldPosition += driver2.getLeftStick().getY() * SHIELD_SPEED;
        }
        if (shieldPosition < INTAKE_SHIELD_UP && shieldPosition < INTAKE_SHIELD_DOWN) {
                shieldPosition = Math.min(INTAKE_SHIELD_UP, INTAKE_SHIELD_DOWN);
        } else if (shieldPosition > INTAKE_SHIELD_UP && shieldPosition > INTAKE_SHIELD_DOWN) {
            shieldPosition = Math.max(INTAKE_SHIELD_UP, INTAKE_SHIELD_DOWN);
        }
        robot.intake.setShield(shieldPosition);

        // shooter
        if (inPowerShotShooterMode) {
            robot.shooter.setShooter(driver2.getRightTrigger().getValue() * SHOOTER_POWERSHOT_POWER);
        } else {
            robot.shooter.setShooter(driver2.getRightTrigger().getValue() * SHOOTER_GOAL_POWER);
        }

        // pusher
        if (driver2.getA().isJustPressed()) {
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
                driver2.getA().update(false);
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
