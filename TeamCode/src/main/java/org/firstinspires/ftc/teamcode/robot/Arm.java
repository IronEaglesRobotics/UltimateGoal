package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.Constants.ARM;
import static org.firstinspires.ftc.teamcode.Constants.CLAW;

// Class for the wobble goal arm and claw
@Config
public class Arm {
    // config variables
    public static double ARM_SPEED = 0.2;
    public static double CLAW_CLOSED = 0.13;
    public static double CLAW_OPEN = 0.7;

    private final DcMotor wobbler;
    private final Servo claw;

    // Constructor
    public Arm(HardwareMap hardwareMap) {
        wobbler = hardwareMap.get(DcMotor.class, ARM);
        claw = hardwareMap.get(Servo.class, CLAW);

        wobbler.setDirection(DcMotor.Direction.REVERSE);
        wobbler.setTargetPosition(0);
        wobbler.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbler.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw.scaleRange(CLAW_CLOSED, CLAW_OPEN);
    }

    // Set claw position
    public void setClaw(Constants.ServoPosition position) {
        claw.setPosition(position == Constants.ServoPosition.OPEN ? 1 : 0);
    }

    // Get claw position
    public Constants.ServoPosition getClaw() {
        return claw.getPosition() > 0.5 ? Constants.ServoPosition.OPEN : Constants.ServoPosition.CLOSED;
    }

    // Set arm position
    public void setArm(int position) {
        wobbler.setTargetPosition(position);
        wobbler.setPower(ARM_SPEED);
    }

    public int getPosition() {
        return wobbler.getCurrentPosition();
    }

    // Reset arm encoder because the movement is based on the number of ticks from the starting position
    public void resetEncoder() {
        wobbler.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbler.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Get Telemetry for the arm and claw
    public String getTelemetry() {
        return String.format(Locale.US, "Arm: %.2f %s \nClaw: %s", wobbler.getPower(), wobbler.getCurrentPosition(), getClaw());
    }
}
