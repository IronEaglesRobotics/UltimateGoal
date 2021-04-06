package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.enums.Position;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE;
import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE_SECONDARY;
import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE_SHIELD;
import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE_SHIELD_DOWN;
import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE_SHIELD_UP;
import static org.firstinspires.ftc.teamcode.util.enums.Position.DOWN;
import static org.firstinspires.ftc.teamcode.util.enums.Position.UP;

public class Intake {
    private final DcMotor intake;
    private final DcMotor secondary;
    private final Servo shield;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, INTAKE);
        secondary = hardwareMap.get(DcMotor.class, INTAKE_SECONDARY);
        shield = hardwareMap.get(Servo.class, INTAKE_SHIELD);

        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        secondary.setDirection(DcMotor.Direction.REVERSE);
        secondary.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setIntake(double power) {
        intake.setPower(power);
        secondary.setPower(power);
    }

    public Position getShield() {
        return Math.abs(shield.getPosition() - INTAKE_SHIELD_UP) < Math.abs(shield.getPosition() - INTAKE_SHIELD_DOWN) ? UP : DOWN;
    }

    public void setShield(Position position) {
        shield.setPosition(position == UP ? INTAKE_SHIELD_UP : INTAKE_SHIELD_DOWN);
    }

    public void setShield(double position) {
        shield.setPosition(position);
    }

    public String getTelemetry() {
        return String.format(Locale.US, "Intake: %.2f %.2f %s", intake.getPower(), secondary.getPower(), getShield());
    }
}
