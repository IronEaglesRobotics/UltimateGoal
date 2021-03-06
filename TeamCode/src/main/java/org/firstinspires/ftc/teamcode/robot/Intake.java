package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.Constants.INTAKE;
import static org.firstinspires.ftc.teamcode.Constants.INTAKE_MAX_SPEED;
import static org.firstinspires.ftc.teamcode.Constants.SECONDARY_INTAKE;
import static org.firstinspires.ftc.teamcode.Constants.SECONDARY_INTAKE_RELATIVE_SPEED;

// Class for the intake
public class Intake {
    private final DcMotor intake;
    private final DcMotor secondary;

    // Constructor
    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, INTAKE);
        secondary = hardwareMap.get(DcMotor.class, SECONDARY_INTAKE);

        // set intake and secondary to opposite directions in order for them to work together
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        secondary.setDirection(DcMotor.Direction.FORWARD);
        secondary.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Set power
    public void setIntake(double power) {
        intake.setPower(power * INTAKE_MAX_SPEED);
        secondary.setPower(power * INTAKE_MAX_SPEED * 0.25);
    }

    // Get Telemetry for the intake
    public String getTelemetry() {
        return String.format(Locale.US, "Intake: %.2f %.2f", intake.getPower(), secondary.getPower());
    }
}
