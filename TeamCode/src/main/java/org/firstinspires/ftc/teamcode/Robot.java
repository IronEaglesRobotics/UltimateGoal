package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode;

public class Robot {
    public MecanumDrive drive;
    public Arm arm;
    private StarterStackDetector stackDetector;
    private DcMotor wheel;

    public Robot(HardwareMap hardwareMap) {
        drive = new MecanumDrive(hardwareMap);
        arm = new Arm(hardwareMap);
        stackDetector = new StarterStackDetector(hardwareMap);

        wheel = hardwareMap.get(DcMotor.class, "wheel");

        wheel.setDirection(DcMotor.Direction.REVERSE);

        wheel.setMode(RunMode.RUN_USING_ENCODER);

        wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setWheel(double power) {
        wheel.setPower(power);
    }

    public double getWheelPower() {
        return wheel.getPower();
    }

    public Telemetry getTelemetry() {
        Telemetry t = new Robot.Telemetry();
        t.setDriveStatus(drive.getTelemetry());
        t.setArmStatus(arm.getTelemetry());
        t.setRingStatus("Ring: "+stackDetector.checkStack());
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
        private String armStatus;
        private String ringStatus;

        public void setDriveStatus(String driveStatus) {
            this.driveStatus = driveStatus;
        }

        public void setArmStatus(String armStatus) {
            this.armStatus = armStatus;
        }

        public void setRingStatus(String ringStatus) {
            this.ringStatus = ringStatus;
        }

        @Override
        public String toString() {
            return String.format("\n%s\n%s\n%s", driveStatus, armStatus, ringStatus);
        }
    }
}
