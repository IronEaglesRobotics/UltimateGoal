package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.ArrayList;
import java.util.Locale;

// Main Autonomous Program
@Autonomous(name = "Auto New", group = "Competition", preselectTeleOp = "Manual")
public class AutoNew extends LinearOpMode {
    private Robot robot;
    private Constants.StarterStack stack;
    private ArrayList<Step> steps;

    // Main method to run all the steps for autonomous
    @Override
    public void runOpMode() {
        // initialize
        telemetry.addLine("Initializing Robot...");
        telemetry.update();
        robot = new Robot(hardwareMap);
        robot.camera.initStackCamera();
        while (robot.camera.getFrameCount() < 1) {
            idle();
        }

        // wait for start
        while (!(isStarted() || isStopRequested())) {
            stack = robot.camera.checkStack();
            telemetry.addLine("Initialized");
            telemetry.addData("Stack", stack);
            telemetry.update();
        }
        robot.camera.stopStackCamera();
        initializeSteps(stack);
//        while(robot.camera.getFrameCount() > 0) {
//            idle();
//        }
//        robot.camera.initTargetingCamera();
        while (robot.camera.getFrameCount() < 1) {
            idle();
        }

        // start up the first step
        int stepNumber = 0;
        double stepTimeout;
        Step step = steps.get(stepNumber);
        stepTimeout = step.getTimeout() != -1 ? getRuntime() + step.getTimeout() : Double.MAX_VALUE;
        step.start();

        // while loop that runs the steps
        while(opModeIsActive() && !isStopRequested()) {
            // once a step finishes move on to the next step
            if (!step.isActive() || getRuntime() > stepTimeout) {
                stepNumber++;
                // if the last step finished break out of the while loop
                if (stepNumber > steps.size() - 1) {
                    break;
                }
                step = steps.get(stepNumber);
                stepTimeout = step.getTimeout() != -1 ? getRuntime() + step.getTimeout() : Double.MAX_VALUE;
                step.start();
            }
            telemetry.addLine(String.format(Locale.US, "Runtime: %.0f", getRuntime()));
            telemetry.addLine("Step number "+stepNumber);
            telemetry.addLine(robot.getTelemetry());
            telemetry.update();
        }

        // stop
        robot.camera.stopTargetingCamera();
    }

    // Load up all of the steps for the autonomous
    private void initializeSteps(Constants.StarterStack stack) {
        steps = new ArrayList<>();
        switch(stack) {
            case NONE:
                steps.add(new Step() {
                    @Override
                    public void start() {
                        robot.arm.setClaw(Constants.ServoPosition.CLOSED);
                    }
                    @Override
                    public boolean isActive() {
                        return false;
                    }
                });
                steps.add(new Step() {
                    @Override
                    public void start() {
                        robot.drive.setTargetPositionRelative(0, 10, 0.5);
                    }
                    @Override
                    public boolean isActive() {
                        return robot.drive.isBusy();
                    }
                });
                break;
            case SINGLE:
                steps.add(new Step() {
                    @Override
                    public void start() {
                        robot.drive.setTargetPositionRelative(0, 10, 0.5);
                    }
                    @Override
                    public boolean isActive() {
                        return robot.drive.isBusy();
                    }
                });
                break;
            case QUAD:
                steps.add(new Step() {
                    @Override
                    public void start() {
                        robot.drive.setTargetPositionRelative(0, 20, 0.5);
                    }
                    @Override
                    public boolean isActive() {
                        return robot.drive.isBusy();
                    }
                });
        }
    }
}