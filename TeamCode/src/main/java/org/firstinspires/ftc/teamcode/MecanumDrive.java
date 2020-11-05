package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

public class MecanumDrive implements DriveBase {
    private final double wheelDiameter = 4.0;
    private final double wheelCircumference = Math.PI * wheelDiameter;
    private final double ticksPerRev;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    public MecanumDrive(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        MecanumDrive drive = this;

        this.ticksPerRev = this.frontLeft.getMotorType().getTicksPerRev();

        this.setRunMode(RunMode.RUN_USING_ENCODER);
        this.setBrakeMode(ZeroPowerBehavior.BRAKE);
    }

    public boolean isBusy() {
        return this.getRunMode() != RunMode.RUN_TO_POSITION
                || this.frontLeft.isBusy()
                || this.frontRight.isBusy()
                || this.backLeft.isBusy()
                || this.backRight.isBusy();
    }

    public void setTargetPositionRelative(double inches, double speed) {
        int ticks = (int)((inches / wheelCircumference) * 560);
        this.setRunMode(RunMode.STOP_AND_RESET_ENCODER);
        this.setRunMode(RunMode.RUN_TO_POSITION);

        this.frontLeft.setTargetPosition(ticks);
        this.frontRight.setTargetPosition(ticks);
        this.backLeft.setTargetPosition(ticks);
        this.backRight.setTargetPosition(ticks);

        this.setPower(speed);
    }

    public void setTargetStrafePositionRelative(double inches, double power) {
        int ticks = (int)((inches / wheelCircumference) * 560);
        this.setRunMode(RunMode.STOP_AND_RESET_ENCODER);
        this.setRunMode(RunMode.RUN_TO_POSITION);

        this.frontLeft.setTargetPosition(ticks);
        this.frontRight.setTargetPosition(-ticks);
        this.backLeft.setTargetPosition(-ticks);
        this.backRight.setTargetPosition(ticks);

        this.setStrafePower(power);
    }

    public void setForwardTargetPositionRelative(double inches, double power) {
        int ticks = (int)((inches / wheelCircumference) * 560);
        this.setRunMode(RunMode.STOP_AND_RESET_ENCODER);
        this.setRunMode(RunMode.RUN_TO_POSITION);

        this.frontLeft.setTargetPosition(ticks);
        this.frontRight.setTargetPosition(ticks);
        this.backLeft.setTargetPosition(ticks);
        this.backRight.setTargetPosition(ticks);

        this.setPower(power);
    }

    public void setPower(double power) {
        this.frontLeft.setPower(power);
        this.frontRight.setPower(power);
        this.backLeft.setPower(power);
        this.backRight.setPower(power);
    }

    public void setStrafePower(double power) {
        this.frontLeft.setPower(power);
        this.frontRight.setPower(-power);
        this.backLeft.setPower(-power);
        this.backRight.setPower(power);
    }

    public String motorTelemetry() {
        return ("fl:" + frontLeft.getPower() + "fr:" + frontRight.getPower() + "bl:" + backLeft.getPower() + "br:" + backRight.getPower());
    }

    public void moveToPoint() {}

    @Override
    public void setRunMode(RunMode runMode) {
        this.frontLeft.setMode(runMode);
        this.frontRight.setMode(runMode);
        this.backLeft.setMode(runMode);
        this.backRight.setMode(runMode);
    }

    public RunMode getRunMode() {
        return this.frontLeft.getMode();
    }

    @Override
    public void setBrakeMode(ZeroPowerBehavior brakeMode) {
        this.frontLeft.setZeroPowerBehavior(brakeMode);
        this.frontRight.setZeroPowerBehavior(brakeMode);
        this.backLeft.setZeroPowerBehavior(brakeMode);
        this.backRight.setZeroPowerBehavior(brakeMode);
    }

    @Override
    public void setInputVector(VectorF vector) {
        if (vector.length() >= 3) {
            setInput(vector.get(0), vector.get(1), vector.get(2));
        } else if (vector.length() >= 2) {
            setInput(vector.get(0), vector.get(1), 0);
        } else {
            setInput(vector.get(0), 0, 0);
        }
    }

    @Override
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
}