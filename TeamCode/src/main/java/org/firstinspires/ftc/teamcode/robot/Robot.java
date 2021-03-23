package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

// Robot class that contains everything on the robot
public class Robot {
    public SampleMecanumDrive drive;
    public Arm arm;
    public Intake intake;
    public Shooter shooter;
    public Camera camera;

    // Constructor
    public Robot(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);
        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        camera = new Camera(hardwareMap);
    }

    // Get Telemetry for the robot
    public String getTelemetry() {
        return arm.getTelemetry() + "\n" +
                intake.getTelemetry() + "\n" +
                shooter.getTelemetry() + "\n" +
                camera.getTelemetry();
    }
}
