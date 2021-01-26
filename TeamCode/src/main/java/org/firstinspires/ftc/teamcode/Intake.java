package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private final DcMotor intake;
    private final DcMotor secondary;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        secondary = hardwareMap.get(DcMotor.class, "secondary");

        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        secondary.setDirection(DcMotor.Direction.FORWARD);
        secondary.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Intake power setter.
    public void setIntake(double power) {
        intake.setPower(power);
        secondary.setPower(power*0.5);
    }

    //Intake telemetry.
    public String getTelemetry() {
        return ("Intake: " + intake.getPower() + " " + secondary.getPower());
    }
}
