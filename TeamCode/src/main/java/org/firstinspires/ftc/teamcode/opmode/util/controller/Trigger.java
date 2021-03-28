package org.firstinspires.ftc.teamcode.opmode.util.controller;

public class Trigger {
    double value = 0;

    public Trigger() {}

    public void update(double value) {
        this.value = value;
    }

    public double getValue() {
        return value;
    }
}