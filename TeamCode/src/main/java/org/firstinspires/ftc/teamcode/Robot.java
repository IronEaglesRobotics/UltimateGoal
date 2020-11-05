package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    public MecanumDrive drive;
    private StarterStackDetector stackDetector;
    private DcMotor leftBackDrive;
    private DcMotor leftFrontDrive;
    private DcMotor rightBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor wobbler;
    private Servo claw;

    public static final double ARM_POWER = 0.3;
    private static final double CLAW_MIN = 0.2;
    private static final double CLAW_MAX = 0.55;

    public Robot(HardwareMap hardwareMap) {
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        wobbler = hardwareMap.get(DcMotor.class, "wobbler");
        claw = hardwareMap.get(Servo.class, "claw");
        claw.scaleRange(CLAW_MIN, CLAW_MAX);

        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        wobbler.setDirection(DcMotor.Direction.FORWARD);

        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wobbler.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wobbler.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    public Telemetry getTelemetry() {
        Telemetry t = new Robot.Telemetry();
        t.setDriveStatus(drive.motorTelemetry());
        t.setArmStatus(wobbler.getPower());
        t.setClawStatus(getClaw());
        t.setRingStatus(stackDetector.checkStack());
        return t;
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
