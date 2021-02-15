package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.ArrayList;
import java.util.Locale;

import static org.firstinspires.ftc.teamcode.Constants.AUTO_AIM_OFFSET_X;
import static org.firstinspires.ftc.teamcode.Constants.POWERSHOT_SHOOTER_POWER;
import static org.firstinspires.ftc.teamcode.Constants.SHOOTER_POWER;

// Main Autonomous Program
@Autonomous(name = "Competition Autonomous", group = "Competition", preselectTeleOp = "Competition TeleOp")
public class Auto extends LinearOpMode {
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
        while(robot.camera.getFrameCount() < 1) {
            idle();
        }

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
                // do the finishing move
                step.end();
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
    }

    // Load up all of the steps for the autonomous
    private void initializeSteps(Constants.StarterStack stack) {
        steps = new ArrayList<>();
        switch(stack) {
            case NONE:
                // reset servos
                addClaw(0, Constants.ServoPosition.CLOSED);
                addPusher(0, Constants.ServoPosition.OPEN);
                // move to wobble goal square
                addMovement(0, -50, 0.5);
                // set down wobble goal
                addShooter(0, POWERSHOT_SHOOTER_POWER);
                addArm(2, Constants.ArmPosition.DOWN);
                addClaw(0.4, Constants.ServoPosition.OPEN);
                addArm(0.4, Constants.ArmPosition.DEFAULT);
                // turn and head to shoot powershots
                addTurn(180);
                addMovement(-39, 6, 0.6);
                addShootPowershots(10);
                addShooter(0, 0);
                // park on white line
                addMovement(0, 8, 0.5);
                addStopTargetingCamera();
                break;
            case SINGLE:
                // reset servos
                addClaw(0, Constants.ServoPosition.CLOSED);
                addPusher(0, Constants.ServoPosition.OPEN);
                // move to wobble goal square
                addMovement(0, -75, 0.5);
                addMovement(26, 0, 0.5);
                // set down wobble goal
                addShooter(0, POWERSHOT_SHOOTER_POWER);
                addArm(2, Constants.ArmPosition.DOWN);
                addClaw(0.4, Constants.ServoPosition.OPEN);
                addArm(0.4, Constants.ArmPosition.DEFAULT);
                // turn and head to shoot powershots
                addTurn(180);
                addMovement(-13, -20, 0.5);
                addShootPowershots(10);
                addShooter(0, 0);
                // park on white line
                addMovement(0, 8, 0.5);
                addStopTargetingCamera();
                break;
            case QUAD:
                // reset servos
                addClaw(0, Constants.ServoPosition.CLOSED);
                addPusher(0, Constants.ServoPosition.OPEN);
                // move to wobble goal squares
                addMovement(0, -95, 0.5);
                // set down wobble goal
                addShooter(0, POWERSHOT_SHOOTER_POWER);
                addArm(2, Constants.ArmPosition.DOWN);
                addClaw(0.4, Constants.ServoPosition.OPEN);
                addArm(0.4, Constants.ArmPosition.DEFAULT);
                // turn and head to shoot powershots
                addTurn(180);
                addMovement(-39, -42, 0.5);
                addShootPowershots(10);
                addShooter(0, 0);
                // park on white line
                addMovement(0, 8, 0.5);
                addStopTargetingCamera();
        }
    }

    // Functions to add steps
    private void addDelay(double timeout) {
        steps.add(new Step(timeout) {
            @Override
            public void start() {}
            @Override
            public void whileRunning() {}
            @Override
            public void end() {}
            @Override public boolean isActive() {
                return true;
            }
        });
    }
    private void addMovement(final double xx, final double yy, final double zz) {
        steps.add(new Step() {
            @Override
            public void start() {
                this.x = xx;
                this.y = yy;
                this.z = zz;
                robot.drive.setTargetPositionRelative(x, y, z);
            }
            @Override
            public void whileRunning() {}
            @Override
            public void end() {}
            @Override public boolean isActive() {
                return robot.drive.isBusy();
            }
        });
    }
    private void addTurn(final double degrees) {
        steps.add(new Step() {
            @Override
            public void start() {
                degreesToTurn = degrees;
                robot.imu.resetGyroHeading();
                heading = 5;
            }
            @Override
            public void whileRunning() {
                robot.drive.setInput(0, 0, -(Math.max((degreesToTurn-heading)/degreesToTurn*0.85,0.3)));
                heading = robot.imu.getGyroHeading360();
            }
            @Override
            public void end() {
                robot.drive.setInput(0, 0, 0);
            }
            @Override
            public boolean isActive() {
                return heading < degreesToTurn - 4 || heading > 360 - 4;
            }
        });
    }
    private void addResetPositionBackwards() {
        steps.add(new Step() {
            @Override
            public void start() {
                robot.imu.resetGyroHeadingToInitial();
                heading = robot.imu.getGyroHeading360();
            }
            @Override
            public void whileRunning() {
                robot.drive.setInput(0, 0, -0.2);
                heading = robot.imu.getGyroHeading360();
            }
            @Override
            public void end() {
                robot.drive.setInput(0, 0, 0);
            }
            @Override
            public boolean isActive() {
                return heading < 180 - 5;
            }
        });
    }
    private void addArm(double timeout, final Constants.ArmPosition position) {
        steps.add(new Step(timeout) {
            @Override
            public void start() {
                robot.arm.setArm(position);
            }
            @Override
            public void whileRunning() {}
            @Override
            public void end() {}
            @Override
            public boolean isActive() {
                return robot.arm.isBusy();
            }
        });
    }
    private void addClaw(double timeout, final Constants.ServoPosition position) {
        steps.add(new Step(timeout) {
            @Override
            public void start() {
                robot.arm.setClaw(position);
            }
            @Override
            public void whileRunning() {}
            @Override
            public void end() {}
            @Override
            public boolean isActive() {
                return true;
            }
        });
    }
    private void addPusher(double timeout, final Constants.ServoPosition position) {
        steps.add(new Step(timeout) {
            @Override
            public void start() {
                robot.shooter.setPusher(position);
            }
            @Override
            public void whileRunning() {}
            @Override
            public void end() {}
            @Override
            public boolean isActive() {
                return true;
            }
        });
    }
    private void addShooter(double timeout, final double power) {
        steps.add(new Step(timeout) {
            @Override
            public void start() {
                robot.shooter.setShooter(power);
            }
            @Override
            public void whileRunning() {}
            @Override
            public void end() {}
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
                powershotsKnockedDown = false;
                ringsFired = 0;
                z = 0;
                aimedAtPowershots = false;
                aimedAtGoal = false;
                firing = false;
            }
            @Override
            public void whileRunning() {
                if (!powershotsKnockedDown) {
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
                            powershotsKnockedDown = true;
                            robot.shooter.setShooter(SHOOTER_POWER);
                            shooterSpeedUpTime = getRuntime();
                        }
                        robot.drive.setInput(0, 0, z);
                    }
                    if (aimedAtPowershots) {
                        robot.shooter.setPusher(Constants.ServoPosition.CLOSED);
                        firing = true;
                        zig = true;
                        zag = false;
                        zigTime = getRuntime();
                        aimedAtPowershots = false;
                    }
                } else if (getRuntime() > shooterSpeedUpTime + 0.85) {
                    if (!firing) {
                        red = robot.camera.getRed();
                        if (red.isValid()) {
                            double px = red.getCenter().x+AUTO_AIM_OFFSET_X;
                            if (Math.abs(px) < 50) {
                                double zMaxSpeed = 0.7;
                                double zErr = Math.abs(px);
                                double zSpeed = (zErr / 50) * zMaxSpeed;
                                if (zErr <= 1) {
                                    z = 0;
                                    aimedAtGoal = true;
                                } else {
                                    z = Math.copySign(zSpeed, px);
                                    aimedAtGoal = false;
                                }
                            }
                        } else  {
                            aimedAtGoal = false;
                        }
                        robot.drive.setInput(0, 0, z);
                    }
                    if (aimedAtGoal) {
                        robot.shooter.setPusher(Constants.ServoPosition.CLOSED);
                        firing = true;
                        zig = true;
                        zag = false;
                        zigTime = getRuntime();
                        aimedAtGoal = false;
                    }
                }
                if (firing) {
                    if (zig && getRuntime() > zigTime + 0.85) {
                        robot.shooter.setPusher(Constants.ServoPosition.OPEN);
                        zig = false;
                        zag = true;
                        zagTime = getRuntime();
                    } else if (zag && getRuntime() > zagTime + 0.85) {
                        firing = false;
                        ringsFired++;
                    }
                }
            }
            @Override
            public void end() {}
            @Override
            public boolean isActive() {
                return ringsFired < 3;
            }
        });
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
            public void end() {}
            @Override
            public boolean isActive() {
                return robot.camera.getFrameCount() < 1;
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
            public void end() {}
            @Override
            public boolean isActive() {
                return robot.camera.getFrameCount() > 0;
            }
        });
    }
}