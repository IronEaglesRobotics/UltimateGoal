package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.enums.Position;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.util.Configurables.R_PUSHER_CLOSED;
import static org.firstinspires.ftc.teamcode.util.Configurables.R_PUSHER_OPEN;
import static org.firstinspires.ftc.teamcode.util.Constants.PUSHER;
import static org.firstinspires.ftc.teamcode.util.Constants.SHOOTER;
import static org.firstinspires.ftc.teamcode.util.enums.Position.CLOSED;
import static org.firstinspires.ftc.teamcode.util.enums.Position.OPEN;

public class Shooter {
    private final DcMotor wheel;
    private final Servo pusher;

    public Shooter(HardwareMap hardwareMap) {
        wheel = hardwareMap.get(DcMotor.class, SHOOTER);
        pusher = hardwareMap.get(Servo.class, PUSHER);

        wheel.setDirection(DcMotor.Direction.REVERSE);
        wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public Position getPusher() {
        return Math.abs(pusher.getPosition() - R_PUSHER_OPEN) > Math.abs(pusher.getPosition() - R_PUSHER_CLOSED) ? OPEN : CLOSED;
    }

    public void setPusher(Position position) {
        pusher.setPosition(position == OPEN ? R_PUSHER_OPEN : R_PUSHER_CLOSED);
    }

    public void setShooter(double power) {
        wheel.setPower(power);
    }

    public double getShooter() {
        return wheel.getPower();
    }

    public String getTelemetry() {
        return String.format(Locale.US, "Shooter: %.2f\nPusher: %s", wheel.getPower(), getPusher());
    }
}
