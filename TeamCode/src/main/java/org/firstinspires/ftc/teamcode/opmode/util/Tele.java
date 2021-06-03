package org.firstinspires.ftc.teamcode.opmode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmode.util.controller.Controller;
import org.firstinspires.ftc.teamcode.util.enums.Alliance;
import org.firstinspires.ftc.teamcode.vision.Detection;

import static org.firstinspires.ftc.teamcode.hardware.Lights.BLUE_AIMED_AND_READY;
import static org.firstinspires.ftc.teamcode.hardware.Lights.BLUE_AIMING;
import static org.firstinspires.ftc.teamcode.hardware.Lights.BLUE_LOCKED_ON;
import static org.firstinspires.ftc.teamcode.hardware.Lights.BLUE_NORMAL;
import static org.firstinspires.ftc.teamcode.hardware.Lights.RED_AIMED_AND_READY;
import static org.firstinspires.ftc.teamcode.hardware.Lights.RED_AIMING;
import static org.firstinspires.ftc.teamcode.hardware.Lights.RED_LOCKED_ON;
import static org.firstinspires.ftc.teamcode.hardware.Lights.RED_NORMAL;
import static org.firstinspires.ftc.teamcode.util.Configurables.ARM_DEFAULT_POS;
import static org.firstinspires.ftc.teamcode.util.Configurables.ARM_DOWN_POS;
import static org.firstinspires.ftc.teamcode.util.Configurables.ARM_SPEED;
import static org.firstinspires.ftc.teamcode.util.Configurables.ARM_UP_POS;
import static org.firstinspires.ftc.teamcode.util.Configurables.AUTO_AIM_ACCEPTABLE_ERROR;
import static org.firstinspires.ftc.teamcode.util.Configurables.AUTO_AIM_MIN_POWER;
import static org.firstinspires.ftc.teamcode.util.Configurables.AUTO_AIM_OFFSET_X;
import static org.firstinspires.ftc.teamcode.util.Configurables.AUTO_AIM_PID;
import static org.firstinspires.ftc.teamcode.util.Configurables.INTAKE_SHIELD_DOWN;
import static org.firstinspires.ftc.teamcode.util.Configurables.INTAKE_SHIELD_SPEED;
import static org.firstinspires.ftc.teamcode.util.Configurables.INTAKE_SHIELD_UP;
import static org.firstinspires.ftc.teamcode.util.Configurables.INTAKE_SPEED;
import static org.firstinspires.ftc.teamcode.util.Configurables.PUSHER_DELAY;
import static org.firstinspires.ftc.teamcode.util.Configurables.SHOOTER_GOAL_POWER;
import static org.firstinspires.ftc.teamcode.util.Configurables.SHOOTER_POWERSHOT_POWER;
import static org.firstinspires.ftc.teamcode.util.enums.Position.CLOSED;
import static org.firstinspires.ftc.teamcode.util.enums.Position.OPEN;

public abstract class Tele extends OpMode {
    public Alliance alliance;
    private Robot robot;

    private Controller driver1;
    private Controller driver2;

    private int armPosition;
    public double shieldPosition;

    private double shooterPower = SHOOTER_GOAL_POWER;
    private double finishTime;
    private boolean checkPusher;
    private boolean zig;

    PIDFController controller;

    public void setAlliance() {
        alliance = Alliance.RED;
    }

    // Init
    @Override
    public void init() {
        telemetry.addLine("Initializing Robot...");
        telemetry.update();

        setAlliance();

        robot = new Robot(hardwareMap, alliance);
        robot.camera.initTargetingCamera();

        robot.arm.resetEncoder();
        robot.arm.setClaw(CLOSED);
        armPosition = ARM_DEFAULT_POS;
        robot.shooter.setPusher(OPEN);
        shieldPosition = INTAKE_SHIELD_UP;
        robot.intake.setShield(shieldPosition);

        controller = new PIDFController(0, 0, 0, 0);

        // set current position of the robot
        robot.drive.setPoseEstimate(PoseStorage.currentPose);

        // set gamepads
        driver1 = new Controller(gamepad1);
        driver2 = new Controller(gamepad2);

        if (alliance == Alliance.RED) {
            robot.lights.setPattern(RED_NORMAL);
        } else if (alliance == Alliance.BLUE) {
            robot.lights.setPattern(BLUE_NORMAL);
        }
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
        boolean isAiming = false;
        Detection red = robot.camera.getRed();
        Detection redPowershot = robot.camera.getRedPowershots().getLeftMost();
        Detection blue = robot.camera.getBlue();
        Detection bluePowershot = robot.camera.getBluePowershots().getLeftMost();
        if (this.alliance == Alliance.RED) {
            // powershots
            if (driver2.getX().isPressed()) {
                isAiming = true;
                shooterPower = SHOOTER_POWERSHOT_POWER;
                double targetPos = redPowershot.getCenter().x + AUTO_AIM_OFFSET_X;
                controller.setPIDF(AUTO_AIM_PID.p, AUTO_AIM_PID.i, AUTO_AIM_PID.d, AUTO_AIM_PID.f);
                controller.setTolerance(AUTO_AIM_ACCEPTABLE_ERROR);
                double output = -controller.calculate(0, targetPos);
                z = Math.abs(controller.getPositionError()) <= AUTO_AIM_ACCEPTABLE_ERROR ? 0 : Math.copySign(Math.max(AUTO_AIM_MIN_POWER, Math.abs(output)), output);
            }
            // goal
            else if (driver2.getY().isPressed()) {
                isAiming = true;
                shooterPower = SHOOTER_GOAL_POWER;
                double targetPos = red.getCenter().x + AUTO_AIM_OFFSET_X;
                controller.setPIDF(AUTO_AIM_PID.p, AUTO_AIM_PID.i, AUTO_AIM_PID.d, AUTO_AIM_PID.f);
                controller.setTolerance(AUTO_AIM_ACCEPTABLE_ERROR);
                double output = -controller.calculate(0, targetPos);
                z = Math.abs(controller.getPositionError()) <= AUTO_AIM_ACCEPTABLE_ERROR ? 0 : Math.copySign(Math.max(AUTO_AIM_MIN_POWER, Math.abs(output)), output);
            }
            else {
                isAiming = false;
                controller.reset();
            }
        } else if (this.alliance == Alliance.BLUE) {
            // powershots
            if (driver2.getX().isPressed()) {
                isAiming = true;
                shooterPower = SHOOTER_POWERSHOT_POWER;
                double targetPos = bluePowershot.getCenter().x + AUTO_AIM_OFFSET_X;
                controller.setPIDF(AUTO_AIM_PID.p, AUTO_AIM_PID.i, AUTO_AIM_PID.d, AUTO_AIM_PID.f);
                controller.setTolerance(AUTO_AIM_ACCEPTABLE_ERROR);
                double output = -controller.calculate(0, targetPos);
                z = Math.abs(controller.getPositionError()) <= AUTO_AIM_ACCEPTABLE_ERROR ? 0 : Math.copySign(Math.max(AUTO_AIM_MIN_POWER, Math.abs(output)), output);
            }
            // goal
            else if (driver2.getY().isPressed()) {
                isAiming = true;
                shooterPower = SHOOTER_GOAL_POWER;
                double targetPos = blue.getCenter().x + AUTO_AIM_OFFSET_X;
                controller.setPIDF(AUTO_AIM_PID.p, AUTO_AIM_PID.i, AUTO_AIM_PID.d, AUTO_AIM_PID.f);
                controller.setTolerance(AUTO_AIM_ACCEPTABLE_ERROR);
                double output = -controller.calculate(0, targetPos);
                z = Math.abs(controller.getPositionError()) <= AUTO_AIM_ACCEPTABLE_ERROR ? 0 : Math.copySign(Math.max(AUTO_AIM_MIN_POWER, Math.abs(output)), output);
            }
            else {
                isAiming = false;
                controller.reset();
            }
        }
        robot.drive.setWeightedDrivePower(new Pose2d(x, y, z));
        robot.drive.update();

        //lights!
        if ((this.alliance == Alliance.RED && (Math.abs(red.getCenter().x + AUTO_AIM_OFFSET_X) <= AUTO_AIM_ACCEPTABLE_ERROR ||
                Math.abs(redPowershot.getCenter().x + AUTO_AIM_OFFSET_X) <= AUTO_AIM_ACCEPTABLE_ERROR)) ||
                (this.alliance == Alliance.BLUE && (Math.abs(blue.getCenter().x + AUTO_AIM_OFFSET_X) <= AUTO_AIM_ACCEPTABLE_ERROR ||
                Math.abs(bluePowershot.getCenter().x + AUTO_AIM_OFFSET_X) <= AUTO_AIM_ACCEPTABLE_ERROR))) {
            if (Math.abs(robot.shooter.getShooter() - shooterPower) <= 0.01) {
                if (alliance == Alliance.RED) {
                    robot.lights.setPattern(RED_AIMED_AND_READY);
                } else if (alliance == Alliance.BLUE) {
                    robot.lights.setPattern(BLUE_AIMED_AND_READY);
                }
            } else {
                if (alliance == Alliance.RED) {
                    robot.lights.setPattern(RED_LOCKED_ON);
                } else if (alliance == Alliance.BLUE) {
                    robot.lights.setPattern(BLUE_LOCKED_ON);
                }
            }
        } else if (isAiming) {
            if (alliance == Alliance.RED) {
                robot.lights.setPattern(RED_AIMING);
            } else if (alliance == Alliance.BLUE) {
                robot.lights.setPattern(BLUE_AIMING);
            }
        } else {
            if (alliance == Alliance.RED) {
                robot.lights.setPattern(RED_NORMAL);
            } else if (alliance == Alliance.BLUE) {
                robot.lights.setPattern(BLUE_NORMAL);
            }
        }

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
            shieldPosition += driver2.getLeftStick().getY() * INTAKE_SHIELD_SPEED;
        }
        if (shieldPosition < INTAKE_SHIELD_UP && shieldPosition < INTAKE_SHIELD_DOWN) {
            shieldPosition = Math.min(INTAKE_SHIELD_UP, INTAKE_SHIELD_DOWN);
        } else if (shieldPosition > INTAKE_SHIELD_UP && shieldPosition > INTAKE_SHIELD_DOWN) {
            shieldPosition = Math.max(INTAKE_SHIELD_UP, INTAKE_SHIELD_DOWN);
        }
        robot.intake.setShield(shieldPosition);

        // shooter
        robot.shooter.setShooter(driver2.getRightTrigger().getValue() * shooterPower);

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
        telemetry.addLine("z: "+z);
        telemetry.addLine("PID values: "+controller.getP()+" "+controller.getI()+" "+controller.getD());
        telemetry.addLine("error: "+controller.getPositionError());
        telemetry.addLine(robot.getTelemetry());
        telemetry.update();
    }

    // Stop function called after TeleOp is finished
    @Override
    public void stop() {
        robot.camera.stopTargetingCamera();
    }
}
