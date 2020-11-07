package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode;


public class Robot {
    public MecanumDrive drive;
    private StarterStackDetector stackDetector;
    private DcMotor leftBackDrive;
    private DcMotor leftFrontDrive;
    private DcMotor rightBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor wobbler;
    private Servo claw;
    private DcMotor wheel;

    public static final double ARM_POWER = 0.3;
    private static final double CLAW_MIN = 0.2;
    private static final double CLAW_MAX = 0.57;

    public Robot(HardwareMap hardwareMap) {
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        wobbler = hardwareMap.get(DcMotor.class, "wobbler");
        wheel = hardwareMap.get(DcMotor.class, "wheel");
        claw = hardwareMap.get(Servo.class, "claw");
        claw.scaleRange(CLAW_MIN, CLAW_MAX);

        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        wobbler.setDirection(DcMotor.Direction.FORWARD);
        wheel.setDirection(DcMotor.Direction.REVERSE);

        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobbler.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel.setMode(RunMode.RUN_USING_ENCODER);

        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbler.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drive = new MecanumDrive(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive);
        stackDetector = new StarterStackDetector(hardwareMap);
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

    public void setArmPosition(double degrees, double power) {
        // ticks for a 20: 537.6
        // ticks for a 40: 1120
        // ticks for a 60: 1680
        int ticks = (int)((degrees/360) * 1120 * 2.75);
        wobbler.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbler.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        wobbler.setTargetPosition(ticks);

        wobbler.setPower(power);
    }

    public void setWheel(double power) {
        wheel.setPower(power);
    }

    public double getWheelPower() {
        return wheel.getPower();
    }

    public boolean isWobblerBusy() {
        return wobbler.getMode() != RunMode.RUN_TO_POSITION || wobbler.isBusy();
    }

    public Telemetry getTelemetry() {
        Telemetry t = new Robot.Telemetry();
        t.setDriveStatus(drive.motorTelemetry());
        t.setArmStatus(wobbler.getPower());
        t.setClawStatus(getClaw());
        t.setRingStatus(stackDetector.checkStack());
        return t;
    }

    public void setTfodZoom(int zoom) {
        this.stackDetector.setZoom(zoom);
    }

    public StarterStackDetector.StarterStack checkStack() {
        return stackDetector.checkStack();
    }

    public class Telemetry {
        private String driveStatus;
        private double armStatus;
        private boolean clawStatus;
        private StarterStackDetector.StarterStack ringStatus;

        public void setDriveStatus(String driveStatus) {
            this.driveStatus = driveStatus;
        }

        public void setArmStatus(double armStatus) {
            this.armStatus = armStatus;
        }

        public void setClawStatus(boolean clawStatus) {
            this.clawStatus = clawStatus;
        }

        public void setRingStatus(StarterStackDetector.StarterStack ringStatus) {
            this.ringStatus = ringStatus;
        }

        public String toString() {
            return String.format("Drive: %s\n Arm: %f\n Claw: %b\n Ring: %s", driveStatus, armStatus, clawStatus, ringStatus);
        }
    }
}
