package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.Constants.INTAKE;
import static org.firstinspires.ftc.teamcode.Constants.INTAKE_SECONDARY;
import static org.firstinspires.ftc.teamcode.Constants.INTAKE_SHIELD;

// Class for the intake
@Config
public class Intake {
    // config variables
    public static double INTAKE_SECONDARY_RELATIVE_SPEED = 0.25;
    public static double INTAKE_SHIELD_UP = 0.41;
    public static double INTAKE_SHIELD_DOWN = 0.94;

    private final DcMotor intake;
    private final DcMotor secondary;
    private final Servo shield;

    // Constructor
    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, INTAKE);
        secondary = hardwareMap.get(DcMotor.class, INTAKE_SECONDARY);
        shield = hardwareMap.get(Servo.class, INTAKE_SHIELD);

        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        secondary.setDirection(DcMotor.Direction.FORWARD);
        secondary.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shield.setDirection(Servo.Direction.REVERSE);
        shield.scaleRange(INTAKE_SHIELD_UP, INTAKE_SHIELD_DOWN);
    }

    // Set intake power
    public void setIntake(double power) {
        intake.setPower(power);
        secondary.setPower(power * INTAKE_SECONDARY_RELATIVE_SPEED);
    }

    // Set shield position
    public void setShield(Constants.ServoPosition position) {
        shield.setPosition(position == Constants.ServoPosition.OPEN ? 1 : 0);
    }

    public void setShield(double position) {
        shield.setPosition(position);
    }

    // Get shield position
    public Constants.ServoPosition getShield() {
        return shield.getPosition() > 0.5 ? Constants.ServoPosition.OPEN : Constants.ServoPosition.CLOSED;
    }

    // Get telemetry for the intake
    public String getTelemetry() {
        return String.format(Locale.US, "Intake: %.2f %.2f %s", intake.getPower(), secondary.getPower(), getShield());
    }
}
