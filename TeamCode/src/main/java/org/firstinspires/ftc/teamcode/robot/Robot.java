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
    public String getTelemetry() {
        return drive.getTelemetry() + "\n" +
                arm.getTelemetry() + "\n" +
                intake.getTelemetry() + "\n" +
                shooter.getTelemetry() + "\n" +
                camera.getTelemetry() + "\n" +
                imu.getTelemetry();
    }
}
