package org.firstinspires.ftc.teamcode.opmodes;

// Class for every step that the autonomous program will take
public abstract class Step {
    private int timeout;

    // Constructors
    public Step() {
        this.timeout = -1;
    }
    public Step(int timeout) {
        this.timeout = timeout;
    }

    // Abstract methods to be overrided
    public abstract void start();
    public abstract boolean isActive();

    // Return the timeout for the step
    public int getTimeout() {
        return timeout;
    }
}