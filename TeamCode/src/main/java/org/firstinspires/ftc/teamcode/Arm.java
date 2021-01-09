package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/*Terms:
* - HardwareMap ... A list of the parts on a robot. Like a phonebook for motors and servos. Going to redefine this later because it is very important.
*/

//For wobble goal grabbing.
public class Arm {
    private final double ticksPerRev;
    private final DcMotor wobbler;
    private final Servo claw;

    //Claw control.
    private static final double CLAW_MIN = 0.35;
    private static final double CLAW_MAX = 0.85;

    //==Constructor==//
    //Gets arm hardware and sets enviornment variables.
    public Arm(HardwareMap hardwareMap) {
        wobbler = hardwareMap.get(DcMotor.class, "wobbler");
        claw = hardwareMap.get(Servo.class, "claw");

        //Set wheel config to forward; run using an encoder for telemetry, and set brake behavior.
        wobbler.setDirection(DcMotor.Direction.FORWARD);
        wobbler.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobbler.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw.scaleRange(CLAW_MIN, CLAW_MAX);

        this.ticksPerRev = this.wobbler.getMotorType().getTicksPerRev();
    }

    //Check if wobbler is running or otherwise busy.
    public boolean isBusy() {
        return this.wobbler.getMode() != DcMotor.RunMode.RUN_TO_POSITION || this.wobbler.isBusy();
    }

    //Claw position setter.
    public void setClaw(boolean open) {
        claw.setPosition(open ? 0 : 1);
    }

    //Claw position getter.
    public boolean getClaw() {
        return claw.getPosition() < 0.5;
    }

    //Wobbler power setter.
    public void setArm(double power) {
        wobbler.setPower(power);
    }

    //
    public void setTargetArmPosition(int degrees, double power) {
        //Get the rotation of your wheels but in ticks instead of degrees, I think.
        int ticks = (int)((degrees / 360.0) * ticksPerRev * 2.75);

        //Reset wobbler and encoder, then give it a new position to track.
        wobbler.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbler.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        wobbler.setTargetPosition(ticks);

        wobbler.setPower(power);
    }

    //Arm and claw telemetry.
    public String getTelemetry() {
        return ("Arm: " + wobbler.getPower() + "\nClaw: " + claw.getPosition());
    }
}

