package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.enums.Alliance;

public class Robot {
    public SampleMecanumDrive drive;
    public Arm arm;
    public Intake intake;
    public Shooter shooter;
    public Camera camera;
    public Lights lights;

    public Robot(HardwareMap hardwareMap, Alliance alliance) {
        drive = new SampleMecanumDrive(hardwareMap);
        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        camera = new Camera(hardwareMap, alliance);
        lights = new Lights(hardwareMap);
    }

    public String getTelemetry() {
        return arm.getTelemetry() + "\n" +
                intake.getTelemetry() + "\n" +
                shooter.getTelemetry() + "\n" +
                camera.getTelemetry() + "\n";
//                +lights.getTelemetry();
    }
}
