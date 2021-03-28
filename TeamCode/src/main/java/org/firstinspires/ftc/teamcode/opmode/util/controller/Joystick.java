package org.firstinspires.ftc.teamcode.opmode.util.controller;

public class Joystick {
    double x = 0;
    double y = 0;

    public Joystick() {}

    public void update(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }
}