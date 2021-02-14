package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.ArrayList;
import java.util.Locale;

import static org.firstinspires.ftc.teamcode.Constants.AUTO_AIM_OFFSET_X;
import static org.firstinspires.ftc.teamcode.Constants.POWERSHOT_SHOOTER_POWER;

// Main Autonomous Program
@Autonomous(name = "Auto New", group = "Competition", preselectTeleOp = "Manual")
public class AutoNew extends LinearOpMode {
    private Robot robot;
    private Constants.StarterStack stack;
    private ArrayList<Step> steps;

    // Main method to run all the steps for autonomous
    @Override
    public void runOpMode() {
        // initialize robot
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

        // switch from stack camera to targeting camera
        robot.camera.stopStackCamera();
        initializeSteps(stack);
        while(robot.camera.getFrameCount() > 0) {
            idle();
        }
        robot.camera.initTargetingCamera();

        // start up the first step
        int stepNumber = 0;
        double stepTimeout;
        Step step = steps.get(stepNumber);
        stepTimeout = step.getTimeout() != -1 ? getRuntime() + step.getTimeout() : Double.MAX_VALUE;
        step.start();

        // run the remaining steps
        while(opModeIsActive() && !isStopRequested()) {
            // once a step finishes
            if (!step.isActive() || getRuntime() > stepTimeout) {
                stepNumber++;
                // if it was the last step break out of the while loop
                if (stepNumber > steps.size() - 1) {
                    break;
                }
                // else continue to the next step
                step = steps.get(stepNumber);
                stepTimeout = step.getTimeout() != -1 ? getRuntime() + step.getTimeout() : Double.MAX_VALUE;
                step.start();
            }

            // while the step is running display telemetry
            step.whileRunning();
            telemetry.addLine(String.format(Locale.US, "Runtime: %.0f", getRuntime()));
            telemetry.addLine("Step "+(stepNumber+1)+" of "+steps.size());
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
                addClaw(0, Constants.ServoPosition.CLOSED);
                addMovement(0, -50, 0.5);
                addShooter(0, POWERSHOT_SHOOTER_POWER);
                addPlaceGoal();
                addTurn(180);
                addMovement(-39, 8, 0.5);
                addShootPowershots(10);
                addShooter(0, 0);
                addMovement(0, 8, 0.5);
                addStopTargetingCamera();
                break;
            case SINGLE:
                addClaw(0, Constants.ServoPosition.CLOSED);
                addMovement(0, -75, 0.5);
                addMovement(26, 0, 0.5);
                addShooter(0, POWERSHOT_SHOOTER_POWER);
                addPlaceGoal();
                addTurn(180);
                addMovement(-13, -18, 0.5);
                addShootPowershots(10);
                addShooter(0, 0);
                addMovement(0, 8, 0.5);
                addStopTargetingCamera();
                break;
            case QUAD:
                addClaw(0, Constants.ServoPosition.CLOSED);
                addMovement(0, -95, 0.5);
                addShooter(0, POWERSHOT_SHOOTER_POWER);
                addPlaceGoal();
                addTurn(180);
                addMovement(-39, 38, 0.5);
                addShootPowershots(10);
                addShooter(0, 0);
                addMovement(0, 8, 0.5);
                addStopTargetingCamera();
        }
    }

    // Functions to add steps
    private void addMovement(final double x, final double y, final double z) {
        steps.add(new Step() {
            @Override
            public void start() {
                robot.drive.setTargetPositionRelative(x, y, z);
            }
            @Override
            public void whileRunning() {}
            @Override
            public boolean isActive() {
                return robot.drive.isBusy();
            }
        });
    }
    private void addTurn(double degrees) {
        steps.add(new Step() {
            @Override
            public void start() {
                degreesToTurn = Math.abs(degrees);
                robot.imu.resetGyroHeading();
                heading = 5;
            }
            @Override
            public void whileRunning() {
                while (heading < degrees - 4 || heading > 360 - 4) {
                    robot.drive.setInput(0, 0, -(Math.max((degrees-heading)/degrees*0.85,0.3)));
                    heading = robot.imu.getGyroHeading360();
                }
                robot.drive.setInput(0,0,0);
            }
            @Override
            public boolean isActive() {
                return robot.drive.isBusy() && degrees > 0;
            }
        });
    }
    private void addArm(double timeout, Constants.ArmPosition position) {
        steps.add(new Step(timeout) {
            @Override
            public void start() {
                robot.arm.setArm(position);
            }
            @Override
            public void whileRunning() {}
            @Override
            public boolean isActive() {
                return false;
            }
        });
    }
    private void addClaw(double timeout, Constants.ServoPosition position) {
        steps.add(new Step(timeout) {
            @Override
            public void start() {
                robot.arm.setClaw(position);
            }
            @Override
            public void whileRunning() {}
            @Override
            public boolean isActive() {
                return true;
            }
        });
    }
    private void addShooter(double timeout, double power) {
        steps.add(new Step(timeout) {
            @Override
            public void start() {
                robot.shooter.setShooter(power);
            }
            @Override
            public void whileRunning() {}
            @Override
            public boolean isActive() {
                return false;
            }
        });
    }
    private void addShootPowershots(double timeout) {
        steps.add(new Step(timeout) {
            @Override
            public void start() {
                ringsFired = 0;
                z = 0;
                aimedAtPowershots = false;
            }
            @Override
            public void whileRunning() {
                if (!firing) {
                    powershot = robot.camera.getPowershots().getLeftMost();
                    if (powershot.isValid()) {
                        double px = powershot.getCenter().x+AUTO_AIM_OFFSET_X;
                        if (Math.abs(px) < 50) {
                            double zMaxSpeed = 0.7;
                            double zErr = Math.abs(px);
                            double zSpeed = (zErr / 50) * zMaxSpeed;
                            if (zErr <= 0.5) {
                                z = 0;
                                aimedAtPowershots = true;
                            } else {
                                z = Math.copySign(zSpeed, px);
                                aimedAtPowershots = false;
                            }
                        }
                    } else  {
                        aimedAtPowershots = false;
                    }
                    robot.drive.setInput(0, 0, z);
                }
                if (aimedAtPowershots) {
                    robot.shooter.setPusher(Constants.ServoPosition.CLOSED);
                    firing = true;
                    zig = true;
                    zigTime = getRuntime();
                }
                if (firing) {
                    if (zig && getRuntime() > zigTime + 850) {
                        robot.shooter.setPusher(Constants.ServoPosition.OPEN);
                        zig = false;
                        zagTime = getRuntime();
                    } else if (getRuntime() > zagTime + 850) {
                        firing = false;
                        ringsFired++;
                    }
                }
            }
            @Override
            public boolean isActive() {
                return ringsFired < 3;
            }
        });
    }
    private void addPlaceGoal() {
        addArm(2, Constants.ArmPosition.DOWN);
        addClaw(.4, Constants.ServoPosition.OPEN);
        addArm(0, Constants.ArmPosition.UP);
    }
    private void addStartTargetingCamera() {
        steps.add(new Step() {
            @Override
            public void start() {
                robot.camera.initTargetingCamera();
            }
            @Override
            public void whileRunning() {}
            @Override
            public boolean isActive() {
                return robot.camera.getFrameCount() < 0;
            }
        });
    }
    private void addStopTargetingCamera() {
        steps.add(new Step() {
            @Override
            public void start() {
                robot.camera.stopTargetingCamera();
            }
            @Override
            public void whileRunning() {}
            @Override
            public boolean isActive() {
                return robot.camera.getFrameCount() > 0;
            }
        });
    }
}