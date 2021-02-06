package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

import static org.firstinspires.ftc.teamcode.Constants.ARM;
import static org.firstinspires.ftc.teamcode.Constants.ARM_DOWN_POS;
import static org.firstinspires.ftc.teamcode.Constants.ARM_UP_POS;
import static org.firstinspires.ftc.teamcode.Constants.CLAW;
import static org.firstinspires.ftc.teamcode.Constants.CLAW_MAX;
import static org.firstinspires.ftc.teamcode.Constants.CLAW_MIN;
import static org.firstinspires.ftc.teamcode.Constants.ARM_SPEED;

// Class for the wobble goal arm
public class Arm {
    private final DcMotor wobbler;
    private final Servo claw;

    // Constructor
    public Arm(HardwareMap hardwareMap) {
        wobbler = hardwareMap.get(DcMotor.class, ARM);
        claw = hardwareMap.get(Servo.class, CLAW);

        // set the wobble arm to move forward with encoders and hold it's position when stopped
        wobbler.setDirection(DcMotor.Direction.FORWARD);
        wobbler.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobbler.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set the range of the claw to the min the max
        claw.scaleRange(CLAW_MIN, CLAW_MAX);
    }

    // Check if the arm is currently moving
    public boolean isBusy() {
        return this.wobbler.getMode() != DcMotor.RunMode.RUN_TO_POSITION || this.wobbler.isBusy();
    }

    // Set claw position
    public void setClaw(Constants.ServoPosition position) {
        claw.setPosition(position == Constants.ServoPosition.OPEN ? 0 : 1);
    }

    // Get claw position
    public Constants.ServoPosition getClaw() {
        return claw.getPosition() < 0.5 ? Constants.ServoPosition.OPEN : Constants.ServoPosition.CLOSED;
    }

    // Set arm position (UP is straight up, DOWN is parallel to the ground)
    public void setArm(Constants.ArmPosition position) {
        wobbler.setTargetPosition(position == Constants.ArmPosition.DOWN ? ARM_DOWN_POS : ARM_UP_POS);
        wobbler.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbler.setPower(ARM_SPEED);
    }

    // Reset arm encoder because the movement is based on the number of ticks from the starting position
    public void resetEncoder() {
        wobbler.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Get Telemetry for the arm and claw
    public String getTelemetry() {
        return ("Arm: " + wobbler.getPower() + " " + wobbler.getCurrentPosition() + "\nClaw: " + claw.getPosition());
    }
}
