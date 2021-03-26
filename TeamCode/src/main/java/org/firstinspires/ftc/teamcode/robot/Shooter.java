package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.Constants.PUSHER;
import static org.firstinspires.ftc.teamcode.Constants.SHOOTER;

// Class for the shooter and pusher
@Config
public class Shooter {
    // config variables
    public static double PUSHER_CLOSED = 0.35;
    public static double PUSHER_OPEN = 0.55;

    private final DcMotor wheel;
    private final Servo pusher;

    // Constructor
    public Shooter(HardwareMap hardwareMap) {
        wheel = hardwareMap.get(DcMotor.class, SHOOTER);
        pusher = hardwareMap.get(Servo.class, PUSHER);

        wheel.setDirection(DcMotor.Direction.REVERSE);
        wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pusher.scaleRange(PUSHER_CLOSED, PUSHER_OPEN);
    }

    // Set the position of the pusher
    public void setPusher(Constants.ServoPosition position) {
        pusher.setPosition(position == Constants.ServoPosition.OPEN ? 1 : 0);
    }

    // Get the position of the pusher
    public Constants.ServoPosition getPusher() {
        return pusher.getPosition() > 0.5 ? Constants.ServoPosition.OPEN : Constants.ServoPosition.CLOSED;
    }

    // Set wheel power
    public void setShooter(double power) {
        wheel.setPower(power);
    }

    // Get Telemetry for the wheel and pusher
    public String getTelemetry() {
        return String.format(Locale.US, "Shooter: %.2f\nPusher: %s", wheel.getPower(), getPusher());
    }
}
