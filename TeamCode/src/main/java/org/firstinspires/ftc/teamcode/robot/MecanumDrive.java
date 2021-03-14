package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import static org.firstinspires.ftc.teamcode.Constants.WHEEL_BACK_LEFT;
import static org.firstinspires.ftc.teamcode.Constants.WHEEL_BACK_RIGHT;
import static org.firstinspires.ftc.teamcode.Constants.WHEEL_CIRCUMFERENCE;
import static org.firstinspires.ftc.teamcode.Constants.WHEEL_FRONT_LEFT;
import static org.firstinspires.ftc.teamcode.Constants.WHEEL_FRONT_RIGHT;

// Class for the mecanum drive base
public class MecanumDrive {
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;
    private final double ticksPerRev;

    // variables to track movement
    private double ticksToMove;
    private boolean trackingFrontLeft;

    // Constructor
    public MecanumDrive(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, WHEEL_FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotor.class, WHEEL_FRONT_RIGHT);
        backLeft = hardwareMap.get(DcMotor.class, WHEEL_BACK_LEFT);
        backRight = hardwareMap.get(DcMotor.class, WHEEL_BACK_RIGHT);

        // reverse some of the motors directions because they are mecanum wheels
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);

        // set motors to use encoders and keep their position when stopped
//        this.setRunMode(RunMode.RUN_USING_ENCODER);
        this.setRunMode(RunMode.RUN_WITHOUT_ENCODER);
        this.setBrakeMode(ZeroPowerBehavior.BRAKE);

        ticksPerRev = frontLeft.getMotorType().getTicksPerRev();
    }

    // Check if the motors are currently moving
    public boolean isBusy() {
        return this.getRunMode() != RunMode.RUN_TO_POSITION
                || this.frontLeft.isBusy()
                || this.frontRight.isBusy()
                || this.backLeft.isBusy()
                || this.backRight.isBusy();
    }

    // Move in two directions a certain number of inches
    public void setTargetPositionRelative(double x, double y, double power) {
        double ticksX = (x / WHEEL_CIRCUMFERENCE) * 615; // These numbers are magicc!!!
        double ticksY = (y / WHEEL_CIRCUMFERENCE) * 615;

        this.setRunMode(RunMode.STOP_AND_RESET_ENCODER);

        // if the 2 directions to go in have the same sign, track the front left, otherwise the back left motor
        // if either one of the directions are 0, then it doesn't matter so make it true just because
        trackingFrontLeft = x * y >= 0;
        // the ticks to move will be Y + X if it is the front left, otherwise the back left is calculated with Y - X
        ticksToMove = Math.abs(trackingFrontLeft ? ticksY + ticksX : ticksY - ticksX);

        this.frontLeft.setTargetPosition((int) (ticksY + ticksX));
        this.frontRight.setTargetPosition((int) (ticksY - ticksX));
        this.backLeft.setTargetPosition((int) (ticksY - ticksX));
        this.backRight.setTargetPosition((int) (ticksY + ticksX));

        this.setRunMode(RunMode.RUN_TO_POSITION);

        this.setPower(power);
    }

    // Get the total number of ticks for the current movement
    public double getTargetDistance() {
        return ticksToMove;
    }

    // Get the number of ticks remaining for the current movement
    public double getTargetDistanceRemaining() {
        return ticksToMove - Math.abs(trackingFrontLeft ? frontLeft.getCurrentPosition() : backLeft.getCurrentPosition());
    }

    // Set wheel power
    public void setPower(double power) {
        this.frontLeft.setPower(power);
        this.frontRight.setPower(power);
        this.backLeft.setPower(power);
        this.backRight.setPower(power);
    }

    // Get wheel power
    public double getPower() {
        return this.frontLeft.getPower();
    }

    // Set the runmode of all the wheels
    public void setRunMode(RunMode runMode) {
        this.frontLeft.setMode(runMode);
        this.frontRight.setMode(runMode);
        this.backLeft.setMode(runMode);
        this.backRight.setMode(runMode);
    }

    // Get the runmode of the wheels (because they should all be the same, just getting the front left should tell what all of them are)
    public RunMode getRunMode() {
        return this.frontLeft.getMode();
    }

    // Set the brakemode of the wheels (whether they should try to hold their position or not mainly)
    public void setBrakeMode(ZeroPowerBehavior brakeMode) {
        this.frontLeft.setZeroPowerBehavior(brakeMode);
        this.frontRight.setZeroPowerBehavior(brakeMode);
        this.backLeft.setZeroPowerBehavior(brakeMode);
        this.backRight.setZeroPowerBehavior(brakeMode);
    }

    // Set the input for the wheels
    public void setInput(double x, double y, double z) {
//        this.setRunMode(RunMode.RUN_USING_ENCODER);

        double flPower, frPower, blPower, brPower;

        flPower = z + x + y;    frPower = -z - x + y;
        blPower = z - x + y;    brPower = -z + x + y;

        double max = (Math.abs(z) + Math.abs(y) + Math.abs(x));

        if (max > 1) {
            flPower /= max; frPower /= max;
            blPower /= max; brPower /= max;
        }

        frontLeft.setPower(flPower);    frontRight.setPower(frPower);
        backLeft.setPower(blPower);     backRight.setPower(brPower);
    }

    public double getTicksPerRev() {
        return ticksPerRev;
    }

    // Get Telemetry of the wheels
    public String getTelemetry() {
        return String.format(Locale.US, "Drive: fl: %.2f fr: %.2f bl: %.2f br: %.2f", frontLeft.getPower(), frontRight.getPower(), backLeft.getPower(), backRight.getPower());
    }
}