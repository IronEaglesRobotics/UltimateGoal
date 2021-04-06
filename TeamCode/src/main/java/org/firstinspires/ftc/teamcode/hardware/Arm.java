package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.enums.Position;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.util.Constants.ARM;
import static org.firstinspires.ftc.teamcode.util.Constants.ARM_ALMOST_DOWN_POS;
import static org.firstinspires.ftc.teamcode.util.Constants.ARM_DEFAULT_POS;
import static org.firstinspires.ftc.teamcode.util.Constants.ARM_DOWN_POS;
import static org.firstinspires.ftc.teamcode.util.Constants.ARM_SPEED;
import static org.firstinspires.ftc.teamcode.util.Constants.ARM_UP_POS;
import static org.firstinspires.ftc.teamcode.util.Constants.CLAW;
import static org.firstinspires.ftc.teamcode.util.Constants.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.util.Constants.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.util.enums.Position.CLOSED;
import static org.firstinspires.ftc.teamcode.util.enums.Position.OPEN;

// Class for the wobble goal arm and claw
public class Arm {
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
    }

    public int getArm() {
        return wobbler.getCurrentPosition();
    }

    public void setArm(int position) {
        wobbler.setTargetPosition(position);
        wobbler.setPower(ARM_SPEED);
    }

    public void setArm(Position position) {
        switch(position) {
            case BACK:
                wobbler.setTargetPosition(ARM_DEFAULT_POS);
                break;
            case UP:
                wobbler.setTargetPosition(ARM_UP_POS);
                break;
            case ALMOST_DOWN:
                wobbler.setTargetPosition(ARM_ALMOST_DOWN_POS);
                break;
            case DOWN:
                wobbler.setTargetPosition(ARM_DOWN_POS);
        }
        wobbler.setPower(ARM_SPEED);
    }

    public void resetEncoder() {
        wobbler.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbler.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public Position getClaw() {
        return Math.abs(claw.getPosition() - CLAW_OPEN) < Math.abs(claw.getPosition() - CLAW_CLOSED) ? OPEN : CLOSED;
    }

    public void setClaw(Position position) {
        claw.setPosition(position == OPEN ? CLAW_OPEN : CLAW_CLOSED);
    }

    public String getTelemetry() {
        return String.format(Locale.US, "Arm: %.2f %s \nClaw: %s", wobbler.getPower(), wobbler.getCurrentPosition(), getClaw());
    }
}
