package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Locale;

@Config
public class Lights {
    public static int NUMBER = 1;
    public static int RED_NORMAL = 51;
    public static int RED_AIMING = 49;
    public static int RED_LOCKED_ON = 84;
    public static int RED_AIMED_AND_READY = 87;
    public static int BLUE_NORMAL = 61;
    public static int BLUE_AIMING = 59;
    public static int BLUE_LOCKED_ON = 84;
    public static int BLUE_AIMED_AND_READY = 87;

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

    public void setPattern(int patternNumber) {
        pattern = RevBlinkinLedDriver.BlinkinPattern.fromNumber(patternNumber);
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
