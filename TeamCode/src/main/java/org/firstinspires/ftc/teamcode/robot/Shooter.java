package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.Constants.PUSHER;
import static org.firstinspires.ftc.teamcode.Constants.PUSHER_MAX;
import static org.firstinspires.ftc.teamcode.Constants.PUSHER_MIN;
import static org.firstinspires.ftc.teamcode.Constants.SHOOTER;

// Class for the shooter
public class Shooter {
    private final DcMotor wheel;
    private final Servo pusher;

    // Constructor
    public Shooter(HardwareMap hardwareMap) {
        wheel = hardwareMap.get(DcMotor.class, SHOOTER);
        pusher = hardwareMap.get(Servo.class, PUSHER);

        // set wheel config to forward; run using an encoder for telemetry, and set brake behavior
        wheel.setDirection(DcMotor.Direction.REVERSE);
        wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // scale the range of the pusher
        pusher.scaleRange(PUSHER_MIN, PUSHER_MAX);
        pusher.setDirection(Servo.Direction.REVERSE);
    }

    // Set position for the pusher
    public void setPusher(Constants.ServoPosition position) {
        pusher.setPosition(position == Constants.ServoPosition.OPEN ? 0 : 1);
    }

    public Constants.ServoPosition getPusher() {
        return pusher.getPosition() < 0.5 ? Constants.ServoPosition.OPEN : Constants.ServoPosition.CLOSED;
    }

    // Set wheel power
    public void setShooter(double power) {
        wheel.setPower(power);
    }

    // Get Telemetry for the wheel
    public String getTelemetry() {
        return String.format(Locale.US, "Shooter: %.2f\nPusher: %s", wheel.getPower(), getPusher());
    }
}
