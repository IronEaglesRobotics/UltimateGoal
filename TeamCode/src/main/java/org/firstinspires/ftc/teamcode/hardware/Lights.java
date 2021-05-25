package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Locale;

@Config
public class Lights {
    public static int NUMBER = 1;
    private RevBlinkinLedDriver blinkinLedDriver;
    private RevBlinkinLedDriver.BlinkinPattern pattern;

    public Lights(HardwareMap hardwareMap) {
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        blinkinLedDriver.setPattern(pattern);
    }

    public void setPattern() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.fromNumber(NUMBER);
        blinkinLedDriver.setPattern(pattern);
    }
    public void nextPattern() {
        pattern = pattern.next();
        blinkinLedDriver.setPattern(pattern);
    }

    public void previousPattern() {
        pattern = pattern.previous();
        blinkinLedDriver.setPattern(pattern);
    }

    public String getTelemetry() {
        return String.format(Locale.US, "Pattern: %s", pattern.toString());
    }
}
