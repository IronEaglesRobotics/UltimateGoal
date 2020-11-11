package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

public class MecanumDrive {
    private final double wheelDiameter = 4.0;
    private final double wheelCircumference = Math.PI * wheelDiameter;
    private final double ticksPerRev;
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;

    public MecanumDrive(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);

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

    public void setTargetTurnPositionRelative(int degrees, double power) {
        int ticks = (int)((degrees / 360.0) * 560);
        this.setRunMode(RunMode.STOP_AND_RESET_ENCODER);
        this.setRunMode(RunMode.RUN_TO_POSITION);

        this.frontLeft.setTargetPosition(ticks);
        this.frontRight.setTargetPosition(-ticks);
        this.backLeft.setTargetPosition(ticks);
        this.backRight.setTargetPosition(-ticks);

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

    public void setTurnPower(double power) {
        this.frontLeft.setPower(power);
        this.frontRight.setPower(-power);
        this.backLeft.setPower(power);
        this.backRight.setPower(-power);
    }

    public void setRunMode(RunMode runMode) {
        this.frontLeft.setMode(runMode);
        this.frontRight.setMode(runMode);
        this.backLeft.setMode(runMode);
        this.backRight.setMode(runMode);
    }

    public RunMode getRunMode() {
        return this.frontLeft.getMode();
    }

    public void setBrakeMode(ZeroPowerBehavior brakeMode) {
        this.frontLeft.setZeroPowerBehavior(brakeMode);
        this.frontRight.setZeroPowerBehavior(brakeMode);
        this.backLeft.setZeroPowerBehavior(brakeMode);
        this.backRight.setZeroPowerBehavior(brakeMode);
    }

    public void setInputVector(VectorF vector) {
        if (vector.length() >= 3) {
            setInput(vector.get(0), vector.get(1), vector.get(2));
        } else if (vector.length() >= 2) {
            setInput(vector.get(0), vector.get(1), 0);
        } else {
            setInput(vector.get(0), 0, 0);
        }
    }

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

    public String getTelemetry() {
        return ("fl:" + frontLeft.getPower() + " fr:" + frontRight.getPower() + " bl:" + backLeft.getPower() + " br:" + backRight.getPower());
    }
}