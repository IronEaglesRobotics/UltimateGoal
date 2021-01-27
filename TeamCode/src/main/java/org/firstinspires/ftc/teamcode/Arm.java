package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/*Terms:
* - HardwareMap ... A list of the parts on a robot. Like a phonebook for motors and servos. Going to redefine this later because it is very important.
*/

//For wobble goal grabbing
public class Arm {
    private final double ticksPerRev;
    private final DcMotor wobbler;
    private final Servo claw;

    //Claw control
    private static final double CLAW_MIN = 0.05;
    private static final double CLAW_MAX = 0.8;

    //Arm control
    private final int ARM_DOWN_POS = -756;
    private final int ARM_UP_POS = -221;

    //==Constructor==//
    //Gets arm hardware and sets environment variables.
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

    //Wobbler power setter. the 2 positions for the arm are straight up in the air and 90 degrees to the ground
    //up will go to the straight up position, and down will go to the 90 degree position
    public void setArm(boolean down) {
        wobbler.setTargetPosition(down ? ARM_DOWN_POS : ARM_UP_POS);
        wobbler.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbler.setPower(0.25);
    }

    // resets the motor encoder at startup
    public void resetEncoder() {
        wobbler.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //Arm and claw telemetry.
    public String getTelemetry() {
        return ("Arm: " + wobbler.getPower() + " " + wobbler.getCurrentPosition() + "\nClaw: " + claw.getPosition());
    }
}

