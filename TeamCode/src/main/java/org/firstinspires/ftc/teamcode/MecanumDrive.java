package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

/*Terms:
* - Mecanum ... Mecanum wheels are those cool omnidirectional wheels we use.
* - Ticks ... Assumably, the smallest distance you can possibly move the wheel.
* - Vector ... A term to know if you don't already. Suggested: https://www.youtube.com/watch?v=fNk_zzaMoSs.
* - RunMode ... The way the wheels are set to run. A few examples are: RUN_USING_ENCODER (run wheels and collect data), STOP_AND_RESET_ENCODER (brake and discard encoder data), RUN_TO_POSITION (drive until you reach a certain position).
* - Encoder ... A thing that collects and stores data about hardware. In our case: wheel data.
*/

//For moving the robot around the playing field.
public class MecanumDrive {
    private final double wheelDiameter = 4.0;
    private final double wheelCircumference = Math.PI * wheelDiameter;
    private final double ticksPerRev;
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;

    //==Constructor==//
    //Initializes mecanum wheels, sets RunMode to use encoders (for PID and/or telemetry).
    public MecanumDrive(HardwareMap hardwareMap) {
        //Locate motors and store in variables.
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        //Reverse some of the motors' cardinal directions since we're dealing with mecanum wheels.
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);

        this.ticksPerRev = this.frontLeft.getMotorType().getTicksPerRev();

        //Use an encoder for the wheels for PID support.
        this.setRunMode(RunMode.RUN_USING_ENCODER);
        this.setBrakeMode(ZeroPowerBehavior.BRAKE);
    }

    //Check to see if the RunMode or any of the wheels are busy.
    public boolean isBusy() {
        return this.getRunMode() != RunMode.RUN_TO_POSITION
                || this.frontLeft.isBusy()
                || this.frontRight.isBusy()
                || this.backLeft.isBusy()
                || this.backRight.isBusy();
    }

    //Set the target position and then tell the robot to run to it at a certain power. This may use PID (hence: STOP_AND_RESET_ENCODER) but I'm not certain about that.
    public void setTargetForwardPositionRelative(double inches, double power) {
        int ticks = (int)((inches / wheelCircumference) * 560);
        this.setRunMode(RunMode.STOP_AND_RESET_ENCODER);
        this.setRunMode(RunMode.RUN_TO_POSITION);

        this.frontLeft.setTargetPosition(ticks);
        this.frontRight.setTargetPosition(ticks);
        this.backLeft.setTargetPosition(ticks);
        this.backRight.setTargetPosition(ticks);

        this.setPower(power);
    }

    //Set the target position and then tell the robot to strafe to it at a certain power.
    public void setTargetStrafePositionRelative(double inches, double power) {
        int ticks = (int)((inches / wheelCircumference) * 560);
        this.setRunMode(RunMode.STOP_AND_RESET_ENCODER);
        this.setRunMode(RunMode.RUN_TO_POSITION);

        this.frontLeft.setTargetPosition(ticks);
        this.frontRight.setTargetPosition(-ticks);
        this.backLeft.setTargetPosition(-ticks);
        this.backRight.setTargetPosition(ticks);

        this.setPower(power);
    }

    //Set wheel power.
    public void setPower(double power) {
        this.frontLeft.setPower(power);
        this.frontRight.setPower(power);
        this.backLeft.setPower(power);
        this.backRight.setPower(power);
    }

    //The the runmode of all the wheels.
    public void setRunMode(RunMode runMode) {
        this.frontLeft.setMode(runMode);
        this.frontRight.setMode(runMode);
        this.backLeft.setMode(runMode);
        this.backRight.setMode(runMode);
    }

    //Fetch the RunMode of the front-left wheel.
    public RunMode getRunMode() {
        return this.frontLeft.getMode();
    }

    //Set wheels to a stop... depending on your argument.
    public void setBrakeMode(ZeroPowerBehavior brakeMode) {
        this.frontLeft.setZeroPowerBehavior(brakeMode);
        this.frontRight.setZeroPowerBehavior(brakeMode);
        this.backLeft.setZeroPowerBehavior(brakeMode);
        this.backRight.setZeroPowerBehavior(brakeMode);
    }

    //??? This is why you need to comment your code.
    public void setInputVector(VectorF vector) {
        if (vector.length() >= 3) {
            setInput(vector.get(0), vector.get(1), vector.get(2));
        } else if (vector.length() >= 2) {
            setInput(vector.get(0), vector.get(1), 0);
        } else {
            setInput(vector.get(0), 0, 0);
        }
    }

    //You guys realize that dividing something by 1 does absolutely nothing, right?
    public void setInput(double x, double y, double z) {
        this.setRunMode(RunMode.RUN_USING_ENCODER);

        double flPower, frPower, blPower, brPower;

        flPower = z + x + y;    frPower = -z - x + y;
        blPower = z - x + y;    brPower = -z + x + y;

        double max = (Math.abs(z) + Math.abs(y) + Math.abs(x));

        if (max < 1) {
            flPower /= 1;   frPower /= 1;
            blPower /= 1;   brPower /= 1;
        } else {
            flPower /= max; frPower /= max;
            blPower /= max; brPower /= max;
        }

        frontLeft.setPower(flPower);    frontRight.setPower(frPower);
        backLeft.setPower(blPower);     backRight.setPower(brPower);
    }

    //Gets power telemetry for all the wheels on the robot.
    public String getTelemetry() {
//        return String.format("\nDrive: %s\nArm: %s\nRing: %s", driveStatus, armStatus, ringStatus);
//        return ("fl: "+frontLeft.getPower()+"\nfr: "+frontRight.getPower()+"\nbl: "+backLeft.getPower()+"\nbr: "+backRight.getPower());
        return String.format(Locale.US, "Drive: fl: %.2f fr: %.2f bl: %.2f br: %.2f", frontLeft.getPower(), frontRight.getPower(), backLeft.getPower(), backRight.getPower());
    }
}