package org.firstinspires.ftc.teamcode.opmode.util.controller;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Controller {
    private final Gamepad gamepad;

    private final Joystick leftStick;
    private final Joystick rightStick;

    private final Button dLeft;
    private final Button dRight;
    private final Button dUp;
    private final Button dDown;

    private final Button a;
    private final Button b;
    private final Button x;
    private final Button y;

    private final Button leftBumper;
    private final Button rightBumper;

    private final Button back;
    private final Button start;

    private final Trigger leftTrigger;
    private final Trigger rightTrigger;

    public Controller(Gamepad gamepad) {
        this.gamepad = gamepad;

        leftStick = new Joystick();
        rightStick = new Joystick();

        dLeft = new Button();
        dRight = new Button();
        dUp = new Button();
        dDown = new Button();

        a = new Button();
        b = new Button();
        x = new Button();
        y = new Button();

        leftBumper = new Button();
        rightBumper = new Button();

        back = new Button();
        start = new Button();

        leftTrigger = new Trigger();
        rightTrigger = new Trigger();
    }

    public void update() {
        leftStick.update(gamepad.left_stick_x, gamepad.left_stick_y);
        rightStick.update(gamepad.right_stick_x, gamepad.right_stick_y);

        dLeft.update(gamepad.dpad_left);
        dRight.update(gamepad.dpad_right);
        dUp.update(gamepad.dpad_up);
        dDown.update(gamepad.dpad_down);

        a.update(gamepad.a);
        b.update(gamepad.b);
        x.update(gamepad.x);
        y.update(gamepad.y);

        leftBumper.update(gamepad.left_bumper);
        rightBumper.update(gamepad.right_bumper);

        back.update(gamepad.back);
        start.update(gamepad.start);

        leftTrigger.update(gamepad.left_trigger);
        rightTrigger.update(gamepad.right_trigger);
    }

    public Joystick getLeftStick() {
        return leftStick;
    }
    public Joystick getRightStick() {
        return rightStick;
    }

    public Button getDLeft() {
        return dLeft;
    }
    public Button getDRight() {
        return dRight;
    }
    public Button getDUp() {
        return dUp;
    }
    public Button getDDown() {
        return dDown;
    }

    public Button getA() {
        return a;
    }
    public Button getB() {
        return b;
    }
    public Button getX() {
        return x;
    }
    public Button getY() {
        return y;
    }

    public Button getLeftBumper() {
        return leftBumper;
    }
    public Button getRightBumper() {
        return rightBumper;
    }

    public Button getBack() {
        return back;
    }
    public Button getStart() {
        return start;
    }

    public Trigger getLeftTrigger() {
        return leftTrigger;
    }
    public Trigger getRightTrigger() {
        return rightTrigger;
    }
}
