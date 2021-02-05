package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*Terms:
* - OpMode ... A series of commands for the robot to follow. Can be interchanged in the control app.
* - Iterative OpMode (or just "OpMode") ... A way to run your code. Abstracts code into stop(), start(), init(), init_loop(), loop() functions. Very often used for TeleOp since it allows asynchronous programming.
* - Linear OpMode ... Another way to run your code. Code is executed linearly and synchronously, just like normal. This is often used for autonomous because it is very straightforward.
*/

// autonomous program
@Autonomous(name = "Auto")
public class Auto extends LinearOpMode {
    private Robot robot;
    private Camera.StarterStack stack;

    public void move(int inches, double power) {
        robot.drive.setTargetForwardPositionRelative(inches, power);
        while(robot.drive.isBusy() && opModeIsActive()) {
            sleep(1);
        }
    }

    public void strafe(int inches, double power) {
        robot.drive.setTargetStrafePositionRelative(inches, power);
        while(robot.drive.isBusy() && opModeIsActive()) {
            sleep(1);
        }
    }

    public void turn(double degrees) {
        final float fudge = 2;
        degrees = Math.abs(degrees);
        robot.resetGyroHeading();
        float current = 5;

        if (degrees > 0) {
            while (current < degrees-fudge || current > 360 - fudge) {
                robot.drive.setInput(0, 0, -(Math.max((degrees-current)/degrees*0.65,0.2)));
                current = robot.getGyroHeading360();

                telemetry.addData("Status", current);
                telemetry.update();
                sleep(1);
            }
        }
        robot.drive.setInput(0,0,0);
    }

    public void shoot() {
        robot.shooter.setPusher(true);
        this.sleep(500);
        robot.shooter.setPusher(false);
        this.sleep(500);
    }

    public void placeGoal() {
        robot.arm.setArm(true);
        while(robot.arm.isBusy() && opModeIsActive()) {
            sleep(1);
        }

        robot.arm.setClaw(false);
        sleep(500);

        move(2, 0.5);

        robot.arm.setArm(false);
        while(robot.arm.isBusy() && opModeIsActive()) {
            sleep(1);
        }
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing Robot");
        telemetry.update();
        robot = new Robot(hardwareMap);
        robot.camera.initStackCamera();

        while (robot.camera.getFrameCount() < 1) {
            this.sleep(1);
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // wait for start
        while (!(isStarted() || isStopRequested())) {
            stack = robot.camera.checkStack();
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Stack", stack);
            telemetry.addData("Size", robot.camera.getSize());
            telemetry.update();
        }

        telemetry.addData("Stack", stack);
        telemetry.update();

        robot.camera.stopStackCamera();

        // movements for auto

        // secure wobble goal
        robot.arm.setClaw(true);
        robot.camera.initTargetingCamera();

        // move up to white line and shoot 3 rings into the high goal
        move(24,0.5);
        robot.arm.setArm(false);

        strafe(12,0.55);
        robot.shooter.setShooter(0.630);
        move(40,0.5);
        strafe(-15,0.5);
        shoot();
        shoot();
        shoot();
        robot.shooter.setShooter(0);
        turn(178);

        // move to drop off wobble goal in correct location and move back to the white line
        switch(stack) {
            case NONE:
                strafe(-8,0.5);
                placeGoal();
                strafe(24,0.5);
                move(-12,0.5);
                break;
            case SINGLE:
                move(-22,0.5);
                strafe(12,0.5);
                placeGoal();
                move(6,0.5);
                break;
            case QUAD:
                move(-46,0.5);
                strafe(-6,0.5);
                placeGoal();
                move(30,0.5);
        }

        robot.camera.stopTargetingCamera();
        telemetry.addData("Status", "Finished");
        telemetry.update();
    }
}