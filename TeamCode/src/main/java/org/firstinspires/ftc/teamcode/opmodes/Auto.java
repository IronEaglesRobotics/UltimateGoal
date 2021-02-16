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
    private boolean stopWasNotRequested;

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
            telemetry.addLine(String.format(Locale.US, "Stack: %s", robot.camera.checkStack()));
            telemetry.addLine(String.format(Locale.US, "Size: %.4f", robot.camera.getStarterStack().getArea()));
            telemetry.update();
        }
        resetStartTime();

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
        while(opModeIsActive()) {
            // if the stop button is pressed shut down the current camera
            if (isStopRequested()) {
                if (stopWasNotRequested) {
                    stopWasNotRequested = false;
                    robot.camera.shutdownCamera();
                }
                if (robot.camera.getFrameCount() > 0) {
                    telemetry.addLine("Shutting down robot...");
                    telemetry.update();
                }
            } else {
                // once a step finishes
                if (step.isFinished() || getRuntime() > stepTimeout) {
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
                telemetry.addLine("Step "+(stepNumber+1)+" of "+steps.size()+", "+step.getTelemetry());
                telemetry.addLine(robot.getTelemetry());
                telemetry.update();
            }
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
                addMovement(0, -55, 0, 0.5);
                // set down wobble goal
                addShooter(0, POWERSHOT_SHOOTER_POWER);
                addArm(2, Constants.ArmPosition.DOWN);
                addClaw(0.4, Constants.ServoPosition.OPEN);
                addArm(0, 0.25); // reset back up
                // turn and head to shoot powershots
                addMovement(-44, 1, 180, 0.6);
                addArm(0, 0);
                addShootPowershots(10);
                addShooter(0, 0);
//                // park on white line
//                addMovement(0, 8, 0.5);
                // turn to pick up second wobble goal
                addArm(0, Constants.ArmPosition.DOWN);
                addMovement(23, -27, 0, 0.5);
                addClaw(.4, Constants.ServoPosition.CLOSED);
                // drop second wobble goal off
                addArm(0, Constants.ArmPosition.UP);
                addMovement(45, -6, 90, 0.5);
                addArm(2, Constants.ArmPosition.DOWN);
                addClaw(0.4, Constants.ServoPosition.OPEN);
                addArm(0, 0.25);
                addMovement(0, 24, 0, 0.5);
                addArm(0, 0);
                addStopTargetingCamera();
                break;
            case SINGLE:
                // reset servos
                addClaw(0, Constants.ServoPosition.CLOSED);
                addPusher(0, Constants.ServoPosition.OPEN);
                // move to wobble goal square
                addMovement(26, -75, 0, 0.5);
                // set down wobble goal
                addShooter(0, POWERSHOT_SHOOTER_POWER);
                addArm(2, Constants.ArmPosition.DOWN);
                addClaw(0.4, Constants.ServoPosition.OPEN);
                addArm(0.4, Constants.ArmPosition.DEFAULT);
                // turn and head to shoot powershots
                addMovement(-22, -20, 180, 0.5);
                addShootPowershots(10);
                addShooter(0, SHOOTER_POWER);
//                // park on white line
//                addMovement(0, 8, 0, 0.5);
                addResetPositionBackwards();
                // strafe to goal
                addStrafeToGoal();
                addArm(0, Constants.ArmPosition.DOWN);
                addIntake(0, 0.5);
                // move backwards
                addMovement(0, -28, 0, 0.5);
                addClaw(0.4, Constants.ServoPosition.CLOSED);
                addArm(3, Constants.ArmPosition.UP);
                // move up to shoot ring into goal
                addMovement(0, 26, 0, 0.5);
                addIntake(0, 0);
                addShootGoal(5, 1);
                addShooter(0, 0);
                // drop off second wobble goal
                addMovement(13, -12, 180, 0.5);
                addArm(2, Constants.ArmPosition.DOWN);
                addClaw(0.4, Constants.ServoPosition.OPEN);
                addArm(3, Constants.ArmPosition.DEFAULT);
                addDelay(5);
                addStopTargetingCamera();
                break;
            case QUAD:
                // reset servos
                addClaw(0, Constants.ServoPosition.CLOSED);
                addPusher(0, Constants.ServoPosition.OPEN);
                // move to wobble goal squares
                addMovement(0, -95, 0, 0.5);
                // set down wobble goal
                addShooter(0, POWERSHOT_SHOOTER_POWER);
                addArm(2, Constants.ArmPosition.DOWN);
                addClaw(0.4, Constants.ServoPosition.OPEN);
                addArm(0.4, Constants.ArmPosition.DEFAULT);
                // turn and head to shoot powershots
                addMovement(-39, -42, 180, 0.5);
                addShootPowershots(10);
                addShooter(0, 0);
//                // park on white line
//                addMovement(0, 8, 0, 0.5);
                addResetPositionBackwards();
                // strafe to goal
                addStrafeToGoal();
                addArm(0, Constants.ArmPosition.DOWN);
                // pick up and fire one ring
                addMovement(0, -20, 0, 0.5);
                addMovement(0, 5, 0, 0.5);
                addIntake(0, 0.5);
                addMovement(0, -7, 0, 0.5);
                addMovement(0, 17, 0, 0.5);
                addShootGoal(5, 1);
                addResetPositionBackwards();
                // pick up remaining rings and wobble goal
                addMovement(0, -21, 0, 0.5);
                addClaw(0.4, Constants.ServoPosition.CLOSED);
                addArm(3, Constants.ArmPosition.UP);
                // move up to shoot rings into goal
                addMovement(0, 26, 0, 0.5);
                addIntake(0, 0);
                addShootGoal(5, 3);
                addShooter(0, 0);
                // drop off second wobble goal
                addMovement(37, 12, 180, 0.5);
                addArm(2, Constants.ArmPosition.DOWN);
                addClaw(0.4, Constants.ServoPosition.OPEN);
                addArm(1, Constants.ArmPosition.DEFAULT);
                addMovement(-24, -24, 0, 0.5);
                addDelay(5);
                addStopTargetingCamera();
        }
    }

    // Functions to add steps
    private void addDelay(double timeout) {
        steps.add(new Step("Waiting for "+timeout+" seconds", timeout) {
            @Override
            public void start() {}
            @Override
            public void whileRunning() {}
            @Override
            public void end() {}
            @Override public boolean isFinished() {
                return false;
            }
        });
    }
    private void addMovement(final double xMovement, final double yMovement, final double degrees, final double speed) {
        if (Math.abs(degrees) > 0) {
            steps.add(new Step("Turning "+degrees+" degrees") {
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
                public boolean isFinished() {
                    return !(heading < degreesToTurn - 4) && !(heading > 360 - 4);
                }
            });
        }
        if (Math.abs(xMovement) > 0 || Math.abs(yMovement) > 0) {
            String telemetry = "Moving ";
            if (Math.abs(xMovement) > 0) {
                telemetry += xMovement > 0 ? xMovement + " right" : xMovement + " left";
                if (Math.abs(yMovement) > 0) {
                    telemetry += " and ";
                }
            }
            if (Math.abs(yMovement) > 0) {
                telemetry += yMovement > 0 ? yMovement + " forward" : xMovement + " back";
            }
            steps.add(new Step(telemetry) {
                @Override
                public void start() {
                    this.x = xMovement;
                    this.y = yMovement;
                    this.power = speed;
                    robot.drive.setTargetPositionRelative(x, y, power);
                }
                @Override
                public void whileRunning() {

                }
                @Override
                public void end() {}
                @Override public boolean isFinished() {
                    return !robot.drive.isBusy();
                }
            });
        }
    }
    private void addArm(double timeout, final Constants.ArmPosition position) {
        steps.add(new Step("Moving arm " + position, timeout) {
            @Override
            public void start() {
                robot.arm.setArm(position);
            }
            @Override
            public void whileRunning() {}
            @Override
            public void end() {}
            @Override
            public boolean isFinished() {
                return !robot.arm.isBusy();
            }
        });
    }
    private void addArm(double timeout, final double power) {
        steps.add(new Step("Setting arm power to " + power, timeout) {
            @Override
            public void start() {
                robot.arm.setArm(power);
            }
            @Override
            public void whileRunning() {}
            @Override
            public void end() {}
            @Override
            public boolean isFinished() {
                return false;
            }
        });
    }
    private void addClaw(double timeout, final Constants.ServoPosition position) {
        steps.add(new Step("Moving claw " + position, timeout) {
            @Override
            public void start() {
                robot.arm.setClaw(position);
            }
            @Override
            public void whileRunning() {}
            @Override
            public void end() {}
            @Override
            public boolean isFinished() {
                return false;
            }
        });
    }
    private void addIntake(double timeout, final double intakePower) {
        steps.add(new Step("Setting intake power to " + intakePower, timeout) {
            @Override
            public void start() {
                robot.intake.setIntake(intakePower);
            }
            @Override
            public void whileRunning() {}
            @Override
            public void end() {}
            @Override
            public boolean isFinished() {
                return false;
            }
        });
    }
    private void addPusher(double timeout, final Constants.ServoPosition position) {
        steps.add(new Step("Setting pusher " + position, timeout) {
            @Override
            public void start() {
                robot.shooter.setPusher(position);
            }
            @Override
            public void whileRunning() {}
            @Override
            public void end() {}
            @Override
            public boolean isFinished() {
                return false;
            }
        });
    }
    private void addShooter(double timeout, final double shooterPower) {
        steps.add(new Step("Setting shooter power to " + shooterPower, timeout) {
            @Override
            public void start() {
                robot.shooter.setShooter(shooterPower);
            }
            @Override
            public void whileRunning() {}
            @Override
            public void end() {}
            @Override
            public boolean isFinished() {
                return false;
            }
        });
    }
    private void addShootPowershots(double timeout) {
        steps.add(new Step("Shooting powershots", timeout) {
            @Override
            public void start() {
                ringsToFire = 3;
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
            public boolean isFinished() {
                return ringsFired >= ringsToFire;
            }
        });
    }
    private void addShootGoal(double timeout, final int rings) {
        steps.add(new Step("Shooting Goal", timeout) {
            @Override
            public void start() {
                powershotsKnockedDown = false;
                ringsFired = 0;
                ringsToFire = rings;
                z = 0;
                aimedAtGoal = false;
                firing = false;
            }
            @Override
            public void whileRunning() {
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
            public boolean isFinished() {
                return ringsFired >= ringsToFire;
            }
        });
    }
    private void addStartTargetingCamera() {
        steps.add(new Step("Starting Targeting Camera") {
            @Override
            public void start() {
                robot.camera.initTargetingCamera();
            }
            @Override
            public void whileRunning() {}
            @Override
            public void end() {}
            @Override
            public boolean isFinished() {
                return robot.camera.getFrameCount() >= 1;
            }
        });
    }
    private void addStopTargetingCamera() {
        steps.add(new Step("Stopping Targeting Camera") {
            @Override
            public void start() {
                robot.camera.stopTargetingCamera();
            }
            @Override
            public void whileRunning() {}
            @Override
            public void end() {}
            @Override
            public boolean isFinished() {
                return robot.camera.getFrameCount() <= 0;
            }
        });
    }

    // Functions to add specific steps
    private void addResetPositionBackwards() {
        steps.add(new Step("Resetting position backwards") {
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
            public boolean isFinished() {
                return heading > 180 - 5;
            }
        });
    }
    private void addStrafeToGoal() {
        steps.add(new Step("Strafing to Goal") {
            @Override
            public void start() {
                aimedAtGoal = false;
            }
            @Override
            public void whileRunning() {
                red = robot.camera.getRed();
                if (red.isValid()) {
                    double px = red.getCenter().x-11;
                    if (Math.abs(px) < 50) {
                        double xMaxSpeed = 0.8;
                        double xErr = Math.abs(px);
                        double xSpeed = (xErr / 50) * xMaxSpeed;
                        if (xErr <= 1) {
                            x = 0;
                            aimedAtGoal = true;
                        } else {
                            x = Math.copySign(xSpeed, px);
                            aimedAtGoal = false;
                        }
                    }
                }
                robot.drive.setInput(x, 0, 0);
            }
            @Override
            public void end() {
                robot.drive.setInput(0, 0, 0);
            }
            @Override
            public boolean isFinished() {
                return aimedAtGoal;
            }
        });
    };
}