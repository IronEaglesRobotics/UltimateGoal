package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

// Robot class that contains everything on the robot
public class Robot {
    public MecanumDrive drive;
    public Arm arm;
    public Intake intake;
    public Shooter shooter;
    public Camera camera;
    public IMU imu;

    // Constructor
    public Robot(HardwareMap hardwareMap) {
        drive = new MecanumDrive(hardwareMap);
        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        camera = new Camera(hardwareMap);
        imu = new IMU(hardwareMap);
    }

    // Get Telemetry for the robot
    public Telemetry getTelemetry() {
        Telemetry t = new Robot.Telemetry();
        t.setDriveStatus(drive.getTelemetry());
        t.setArmStatus(arm.getTelemetry());
        t.setIntakeStatus(intake.getTelemetry());
        t.setShooterStatus(shooter.getTelemetry());
        t.setCameraStatus(camera.getTelemetry());
        return t;
    }

    // Class used in getting the Telemetry
    public class Telemetry {
        private String driveStatus;
        private String armStatus;
        private String intakeStatus;
        private String shooterStatus;
        private String cameraStatus;

        // Set the MecanumDrive Telemetry
        public void setDriveStatus(String driveStatus) {
            this.driveStatus = driveStatus;
        }
        // Set the Arm Telemetry
        public void setArmStatus(String armStatus) {
            this.armStatus = armStatus;
        }

        // Set the Intake Telemetry
        public void setIntakeStatus(String armStatus) {
            this.intakeStatus = armStatus;
        }

        // Set the Shooter Telemetry
        public void setShooterStatus(String armStatus) {
            this.shooterStatus = armStatus;
        }

        // Set the Camera Telemetry
        public void setCameraStatus(String cameraStatus) {
            this.cameraStatus = cameraStatus;
        }

        // Override the toString() method (the toString() is called when the Telemetry is displayed on the screen)
        @Override
        public String toString() {
            return String.format("\n%s\n%s\n%s\n%s\n%s", driveStatus, armStatus, intakeStatus, shooterStatus, cameraStatus);
        }
    }
}
