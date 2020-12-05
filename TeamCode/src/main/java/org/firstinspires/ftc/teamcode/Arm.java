package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/*Terms:
* - HardwareMap ... A list of all the parts on a robot. Like a phonebook for motors and servos. Going to redefine this later because it is very important.
*/

//For wobble goal grabbing.
public class Arm {
    private final double ticksPerRev;
    private final DcMotor wobbler;
    private final Servo claw;

    private static final double CLAW_MIN = 0.2;
    private static final double CLAW_MAX = 0.57;

    public Arm(HardwareMap hardwareMap) {
        wobbler = hardwareMap.get(DcMotor.class, "wobbler");
        claw = hardwareMap.get(Servo.class, "claw");

        wobbler.setDirection(DcMotor.Direction.FORWARD);
        wobbler.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobbler.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw.scaleRange(CLAW_MIN, CLAW_MAX);

        this.ticksPerRev = this.wobbler.getMotorType().getTicksPerRev();
    }

    public boolean isBusy() {
        return this.wobbler.getMode() != DcMotor.RunMode.RUN_TO_POSITION || this.wobbler.isBusy();
    }

    public void setClaw(boolean open) {
        claw.setPosition(open ? 0 : 1);
    }

    public boolean getClaw() {
        return claw.getPosition() < 0.5;
    }

    public void setArm(double power) {
        wobbler.setPower(power);
    }

    public void setTargetArmPosition(int degrees, double power) {
        int ticks = (int)((degrees / 360.0) * ticksPerRev * 2.75);
        wobbler.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbler.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        wobbler.setTargetPosition(ticks);

        wobbler.setPower(power);
    }

    public String getTelemetry() {
        return ("Arm: " + wobbler.getPower() + "\nClaw: " + claw.getPosition());
    }
}

