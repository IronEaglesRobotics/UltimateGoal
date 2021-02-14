package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.opencv.Detection;

// Class for every step that the autonomous program will take
public abstract class Step {
    private final double timeout;

    // variables when turning
    public double degreesToTurn;
    public double heading;

    // variables when shooting
    public int ringsFired;
    public Detection powershot;
    public double z;
    public boolean aimedAtPowershots;
    public boolean firing;
    public boolean zig;
    public double zigTime;
    public double zagTime;

    // Constructors
    public Step() {
        this.timeout = -1;
    }
    public Step(double timeout) {
        this.timeout = timeout;
    }

    // Abstract methods to be overrode
    public abstract void start();
    public abstract void whileRunning();
    public abstract boolean isActive();

    // Return the timeout for the step
    public double getTimeout() {
        return timeout;
    }
}