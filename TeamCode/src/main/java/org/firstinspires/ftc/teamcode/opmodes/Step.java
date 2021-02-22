package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.opencv.Detection;

// Class for every step that the autonomous program will take
public abstract class Step {
    private final double timeout;
    private String telemetry;

    // step variables
    public double ticks;
    public double ticksTraveled;
    public double ticksLeft;

    // variables when turning
    public double destinationHeading;
    public double currentHeading;
    public double xErr;
    public double yErr;
    public double zErr;
    public double zRuntime;
    public double xRuntime;
    public double yRuntime;

    // variables when moving
    public double x;
    public double y;
    public double z;
    public double power;

    // variables when shooting
    public int ringsToFire;
    public boolean powershotsKnockedDown;
    public int ringsFired;
    public boolean firing;
    public Detection powershot;
    public Detection red;
    public boolean aimedAtPowershots;
    public boolean aimedAtGoal;
    public boolean zig;
    public boolean zag;
    public double zigTime;
    public double zagTime;
    public double shooterSpeedUpTime;

    // Constructors
    public Step() {
        this.timeout = -1;
    }
    public Step(double timeout) {
        this.timeout = timeout;
    }
    public Step(String telemetry) {
        this.telemetry = telemetry;
        this.timeout = -1;
    }
    public Step(String telemetry, double timeout) {
        this.telemetry = telemetry;
        this.timeout = timeout;
    }

    // Abstract methods to be overrode
    public abstract void start();
    public abstract void whileRunning();
    public abstract void end();
    public abstract boolean isFinished();

    // Return the timeout for the step
    public double getTimeout() {
        return timeout;
    }

    public void setTelemetry(String telemetry) {
        this.telemetry = telemetry;
    }
    // Return the Telemetry for the step
    public String getTelemetry() {
        return telemetry;
    }
}