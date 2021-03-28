package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.drive.SampleMecanumDrive;

public class Robot {
    public SampleMecanumDrive drive;
    public Arm arm;
    public Intake intake;
    public Shooter shooter;
    public Camera camera;

    public Robot(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);
        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        camera = new Camera(hardwareMap);
    }

    public String getTelemetry() {
        return arm.getTelemetry() + "\n" +
                intake.getTelemetry() + "\n" +
                shooter.getTelemetry() + "\n" +
                camera.getTelemetry();
    }
}
