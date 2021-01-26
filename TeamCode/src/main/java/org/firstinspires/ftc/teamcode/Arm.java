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
    private boolean armIsDown; // state of where the arm is

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
        if (down) {
            armIsDown = true;
            wobbler.setTargetPosition(ARM_DOWN_POS);
        } else {
            armIsDown = false;
            wobbler.setTargetPosition(ARM_UP_POS);
        }
        wobbler.setPower(0.25);
    }

    // keep the arm in the right position
    public void stay() {
        if (!isBusy()) {
            wobbler.setTargetPosition(armIsDown ? ARM_DOWN_POS : ARM_UP_POS);
//            if (armIsDown) {
//                if (wobbler.getCurrentPosition() > ARM_DOWN_POS-20) {
//                    //move up
//                } else if (wobbler.getCurrentPosition() < ARM_DOWN_POS+20) {
//                    //move down
//                }
//            } else {
//                if (wobbler.getCurrentPosition() > ARM_UP_POS-20) {
//                    //move up
//                } else if (wobbler.getCurrentPosition() < ARM_UP_POS+20) {
//                    //move down
//                }
//            }
        }
    }

    public void resetEncoder() {
        wobbler.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbler.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //
    public void setTargetArmPosition(int degrees, double power) {
        //Get the rotation of your wheels but in ticks instead of degrees
        int ticks = (int)((degrees / 360.0) * ticksPerRev * 2.75);

        //Reset wobbler and encoder, then give it a new position to track.
        wobbler.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbler.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        wobbler.setTargetPosition(ticks);

        wobbler.setPower(power);
    }

    //Arm and claw telemetry.
    public String getTelemetry() {
        return ("Arm: " + wobbler.getPower() + " " + wobbler.getCurrentPosition() + "\nClaw: " + claw.getPosition());
    }
}

