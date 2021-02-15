package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.opencv.Detection;

// Class for every step that the autonomous program will take
public abstract class Step {
    private final double timeout;

    // variables when turning
    public double degreesToTurn;
    public double heading;

    // variables when shooting
    public boolean powershotsKnockedDown;
    public int ringsFired;
    public Detection powershot;
    public Detection red;
    public double x;
    public double y;
    public double z;
    public boolean aimedAtPowershots;
    public boolean aimedAtGoal;
    public boolean firing;
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

    // Abstract methods to be overrode
    public abstract void start();
    public abstract void whileRunning();
    public abstract void end();
    public abstract boolean isActive();

    // Return the timeout for the step
    public double getTimeout() {
        return timeout;
    }
}