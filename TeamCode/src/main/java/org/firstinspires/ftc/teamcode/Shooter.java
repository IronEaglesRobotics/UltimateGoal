package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
    private final DcMotor wheel;
    private final Servo pusher;

    //Claw control.
    private static final double PUSHER_MIN = 0.22; // pusher in the hopper
    private static final double PUSHER_MAX = 0.5; // pusher out of the hopper

    //==Constructor==//
    //Gets arm hardware and sets environment variables.
    public Shooter(HardwareMap hardwareMap) {
        wheel = hardwareMap.get(DcMotor.class, "wheel");
        pusher = hardwareMap.get(Servo.class, "pusher");

        //Set wheel config to forward; run using an encoder for telemetry, and set brake behavior.
        wheel.setDirection(DcMotor.Direction.REVERSE);

        pusher.scaleRange(PUSHER_MIN, PUSHER_MAX);
    }

    //Pusher position setter.
    public void setPusher(boolean open) {
        pusher.setPosition(open ? 0 : 1);
    }

    //Pusher position getter.
    public boolean getPusher() {
        return pusher.getPosition() <= 0.27;
    }

    public boolean getOpen() {
        return pusher.getPosition() >= 0.45;
    }

    //Wheel power setter.
    public void setShooter(double power) {
        wheel.setPower(power);
    }

    //Wheel and pusher telemetry.
    public String getTelemetry() {
        return ("Shooter: " + wheel.getPower() + "\nPusher: " + pusher.getPosition());
    }
}
