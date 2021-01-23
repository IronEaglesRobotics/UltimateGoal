package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private final DcMotor intake;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");

        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Intake power setter.
    public void setIntake(double power) {
        intake.setPower(power);
    }

    //Intake telemetry.
    public String getTelemetry() {
        return ("Intake: " + intake.getPower());
    }
}
