package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.Constants.INTAKE;
import static org.firstinspires.ftc.teamcode.Constants.INTAKE_SECONDARY;
import static org.firstinspires.ftc.teamcode.Constants.INTAKE_SECONDARY_RELATIVE_SPEED;

// Class for the intake
public class Intake {
    private final DcMotor intake;
    private final DcMotor secondary;

    // Constructor
    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, INTAKE);
        secondary = hardwareMap.get(DcMotor.class, INTAKE_SECONDARY);

        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        secondary.setDirection(DcMotor.Direction.FORWARD);
        secondary.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Set intake power
    public void setIntake(double power) {
        intake.setPower(power);
        secondary.setPower(power * INTAKE_SECONDARY_RELATIVE_SPEED);
    }

    // Get telemetry for the intake
    public String getTelemetry() {
        return String.format(Locale.US, "Intake: %.2f %.2f", intake.getPower(), secondary.getPower());
    }
}
