package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.ArrayList;
import java.util.Locale;

import static org.firstinspires.ftc.teamcode.Constants.AUTO_AIM_OFFSET_X;
import static org.firstinspires.ftc.teamcode.Constants.ArmPosition.DEFAULT;
import static org.firstinspires.ftc.teamcode.Constants.ArmPosition.DOWN;
import static org.firstinspires.ftc.teamcode.Constants.ArmPosition.UP;
import static org.firstinspires.ftc.teamcode.Constants.POWERSHOT_SHOOTER_POWER;
import static org.firstinspires.ftc.teamcode.Constants.SHOOTER_POWER;
import static org.firstinspires.ftc.teamcode.Constants.WHEEL_SPEED;

// Main Autonomous Program
@Autonomous(name = "Competition Autonomous", group = "Competition", preselectTeleOp = "Competition TeleOp")
public class Auto extends LinearOpMode {
    private Robot robot;
    private Constants.StarterStack stack;
    private ArrayList<Step> steps;
    private double currentRuntime;
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
        stepTimeout = step.getTimeout() != -1 ? currentRuntime + step.getTimeout() : Double.MAX_VALUE;
        step.start();

        // run the remaining steps
        while(opModeIsActive()) {
            currentRuntime = getRuntime();
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
                if (step.isFinished() || currentRuntime >= stepTimeout) {
                    // do the finishing move
                    step.end();
                    stepNumber++;
                    // if it was the last step break out of the while loop
                    if (stepNumber > steps.size() - 1) {
                        break;
                    }
                    // else continue to the next step
                    step = steps.get(stepNumber);
                    stepTimeout = step.getTimeout() != -1 ? currentRuntime + step.getTimeout() : Double.MAX_VALUE;
                    step.start();
                }

                // while the step is running display telemetry
                step.whileRunning();
                telemetry.addLine(String.format(Locale.US, "Runtime: %.0f", currentRuntime));
                telemetry.addLine("Step "+(stepNumber+1)+" of "+steps.size()+", "+step.getTelemetry()+"\n");
                telemetry.addLine(robot.getTelemetry());
                telemetry.update();
            }
        }
    }

    // Load up all of the steps for the autonomous
    private void initializeSteps(Constants.StarterStack stack) {
        steps = new ArrayList<>();
        // reset servos
        addClaw(0, Constants.ServoPosition.OPEN);
        addPusher(0, Constants.ServoPosition.OPEN);
        // move depending on the starter stack
        switch(stack) {
            case NONE:
                // move to wobble goal square
                addMovement(-4, -54, 1);
                addIntake(1, 0.5);
                addIntake(0, 0);
                addMovement(0, 4, 1);
                // release wobble goal and start up the shooter
//                addArm(0, UP);
                addShooter(0, POWERSHOT_SHOOTER_POWER);
                // turn and head to shoot powershots
                addTurnAbsoluteFast(180);
                addMovement(-52, 4, 1);
                // shoot the powershots
                addShootPowershots(10);
                addShooter(0, 0);
                // get in position to pick up second wobble goal
                addTurnAbsolute(180);
                addArm(0, DOWN);
                addMoveToGoal();
                // move back and pick up the second wobble goal
                addMovement(0, -27, 1);
                addClaw(0.4, Constants.ServoPosition.CLOSED);
                addArm(0, Constants.ArmPosition.UP);
                // drop off the second wobble goal while parking on the white line
                addTurnAbsoluteFast(270);
                addMovementWithArm(53, -9, 1);
                // reset arm at the end
                addArm(0, DEFAULT);
                addMovement(-2, 12, 0.5);
                break;
            case SINGLE:
                // move to wobble goal square
                addMovement(18, -78, 1);
                addIntake(1, 0.5);
                addIntake(0, 0);
                addMovement(0, 4, 1);
                // release wobble goal and start up the shooter
                addShooter(0, SHOOTER_POWER);
                // turn and head to shoot powershots
                addTurnAbsoluteFast(180);
                addMovement(0, -17, 1);
                addShootGoal(10, 3);
                // get in position to pick up second wobble goal
                addTurnAbsolute(180);
                addArm(0, DOWN);
                addMoveToGoal();
                // move back and pick up the rings and second wobble goal
                addIntake(0, 0.5);
                addMovement(0, -30, 1);
                addClaw(0.4, Constants.ServoPosition.CLOSED);
                addArm(3, Constants.ArmPosition.UP);
                // move up to shoot ring into goal
                addMovement(0, 26, 1);
                addShootGoal(5, 1);
                addIntake(0, 0);
                addShooter(0, 0);
                // drop off second wobble goal
                addTurnAbsoluteFast(0);
                addMovementWithArm(10, -20, 1);
                addArm(3, Constants.ArmPosition.DEFAULT);
                break;
            case QUAD:
                // move to wobble goal squares
                addMovement(-4, -98, 1);
                addIntake(1, 0.5);
                addIntake(0, 0);
                addMovement(0, 4, 1);
                // set down wobble goal
                addShooter(0, SHOOTER_POWER);
                // turn and head to shoot powershots
                addTurnAbsoluteFast(180);
                addMovement(-18, -41, 1);
                addShootGoal(8, 3);
                addTurnAbsolute(180);
                // strafe to goal
                addArm(0, DOWN);
                addMoveToGoal();
                // pick up and fire one ring
                addMovement(0, -11, 1);
                addIntake(0, 0.5);
                addMovement(0, -2, 1);
                addMovement(0, 15, 1);
                addShootGoal(4, 1);
                addTurnAbsolute(180);
                // pick up remaining rings and wobble goal
                addMovement(0, -26, 0.1);
                addClaw(0.4, Constants.ServoPosition.CLOSED);
                addArm(3, Constants.ArmPosition.UP);
                // move up to shoot rings into goal
                addMovement(0, 27, 1);
                addShootGoal(7, 3);
                addIntake(0, 0);
                addShooter(0, 0);
                // drop off second wobble goal
                addTurnAbsoluteFast(0);
                addMovementWithArm(-16, -36, 1);
                addArm(0, Constants.ArmPosition.DEFAULT);
                addMovement(16, 28, 1);
                addDelay(5);
        }
        // stop the targeting camera
        addStopTargetingCamera();
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
            @Override
            public boolean isFinished() {
                return false;
            }
        });
    }
    private void addMovement(final double xMovement, final double yMovement, final double speed) {
        String status = "Moving ";
        if (Math.abs(xMovement) > 0) {
            status += xMovement > 0 ? xMovement + " right" : Math.abs(xMovement) + " left";
            if (Math.abs(yMovement) > 0) {
                status += " and ";
            }
        }
        if (Math.abs(yMovement) > 0) {
            status += yMovement > 0 ? yMovement + " forward" : Math.abs(yMovement) + " back";
        }
        steps.add(new Step(status) {
            @Override
            public void start() {
                this.x = xMovement;
                this.y = yMovement;
                this.power = speed;
                robot.drive.setTargetPositionRelative(x, y, power);
            }
            @Override
            public void whileRunning() {
                if ((robot.drive.getTargetDistance() - robot.drive.getTargetDistanceRemaining() < 500 ||
                        robot.drive.getTargetDistanceRemaining() < 1250 ) && this.power > 0.5) {
                    robot.drive.setPower(0.5);
                } else if (robot.drive.getTargetDistanceRemaining() > 500 && this.power > 0.5) {
                    robot.drive.setPower(1);
                }
            }
            @Override
            public void end() {}
            @Override
            public boolean isFinished() {
                return !robot.drive.isBusy() || robot.drive.getTargetDistanceRemaining() < 15;
            }
        });
    }
    private void addTurnAbsolute(final double degrees) {
        steps.add(new Step("Turning "+degrees+" degrees") {
            @Override
            public void start() {
                robot.sensors.resetGyroHeadingToInitial();
                destinationHeading = degrees;
                zRuntime = -1;
            }
            @Override
            public void whileRunning() {
                currentHeading = robot.sensors.getGyroHeading360();
                // determine the error (special case because the heading resets to 0 instead of 360)
                // the logic works, but in the future making it clearer and more efficient would be nice
                if (currentHeading > 180 && currentHeading > destinationHeading + 180) {
                    zErr = 360 - currentHeading + destinationHeading;
                } else {
                    zErr = (destinationHeading - currentHeading + 180) % 360 - 180;
                }

                // determine whether to turn or not
                if (Math.abs(zErr) <= 1) {
                    z = 0;
                    if (zRuntime == -1) {
                        zRuntime = currentRuntime;
                    }
                } else {
                    zRuntime = -1;
                    // set the speed proportionally to the error the robot is off by, with a minimum speed of 0.15
                    z = Math.copySign(Math.abs(zErr) > 45 ? 0.7 : 0.15, -zErr);
                }
                robot.drive.setInput(0, 0, z);
            }
            @Override
            public void end() {
                robot.drive.setInput(0, 0, 0);
            }
            @Override
            public boolean isFinished() {
                // if the robot is within a degree of the target position for more than 1 second
                return currentRuntime > zRuntime + 1 && zRuntime != -1;
            }
        });
    }
    private void addTurnAbsoluteFast(final double degrees) {
        steps.add(new Step("Turning "+degrees+" degrees") {
            @Override
            public void start() {
                robot.sensors.resetGyroHeadingToInitial();
                destinationHeading = degrees;
                zRuntime = -1;
            }
            @Override
            public void whileRunning() {
                currentHeading = robot.sensors.getGyroHeading360();
                // determine the error (special case because the heading resets to 0 instead of 360)
                // the logic works, but in the future making it clearer and more efficient would be nice
                if (currentHeading > 180 && currentHeading > destinationHeading + 180) {
                    zErr = 360 - currentHeading + destinationHeading;
                } else {
                    zErr = (destinationHeading - currentHeading + 180) % 360 - 180;
                }

                // determine whether to turn or not
                if (Math.abs(zErr) <= 3) {
                    z = 0;
                    if (zRuntime == -1) {
                        zRuntime = currentRuntime;
                    }
                } else {
                    zRuntime = -1;
                    // set the speed proportionally to the error the robot is off by, with a minimum speed of 0.15
                    z = Math.copySign(Math.abs(zErr) > 45 ? 0.7 : 0.15, -zErr);
                }
                robot.drive.setInput(0, 0, z);
            }
            @Override
            public void end() {
                robot.drive.setInput(0, 0, 0);
            }
            @Override
            public boolean isFinished() {
                // if the robot is within a degree of the target position for more than 1 second
                return z == 0;
            }
        });
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
                return !robot.arm.isBusy(); // this essentially is always returning false I think...
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
//                zRuntime = -1;
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
//                                    if (zRuntime == -1) {
//                                        zRuntime = currentRuntime;
//                                    }
                                    aimedAtPowershots = true;
                                } else {
                                    z = Math.copySign(zSpeed, px);
//                                    zRuntime = -1;
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
//                     && currentRuntime > zRuntime + 0.5
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
                    if (zig && getRuntime() > zigTime + 0.5) {
                        robot.shooter.setPusher(Constants.ServoPosition.OPEN);
                        zig = false;
                        zag = true;
                        zagTime = getRuntime();
                    } else if (zag && getRuntime() > zagTime + 0.5) {
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
                    if (zig && getRuntime() > zigTime + 0.4) {
                        robot.shooter.setPusher(Constants.ServoPosition.OPEN);
                        zig = false;
                        zag = true;
                        zagTime = getRuntime();
                    } else if (zag && getRuntime() > zagTime + 0.4) {
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
    private void addMovementWithArm(final double xMovement, final double yMovement, final double speed) {
        String status = "Moving ";
        if (Math.abs(xMovement) > 0) {
            status += xMovement > 0 ? xMovement + " right" : Math.abs(xMovement) + " left";
            if (Math.abs(yMovement) > 0) {
                status += " and ";
            }
        }
        if (Math.abs(yMovement) > 0) {
            status += yMovement > 0 ? yMovement + " forward" : Math.abs(yMovement) + " back";
        }
        steps.add(new Step(status) {
            @Override
            public void start() {
                this.x = xMovement;
                this.y = yMovement;
                this.power = speed;
                robot.drive.setTargetPositionRelative(x, y, power);
            }
            @Override
            public void whileRunning() {
                if (robot.drive.getTargetDistance() - robot.drive.getTargetDistanceRemaining() < 500 ||
                        robot.drive.getTargetDistanceRemaining() < 1250) {
                    robot.drive.setPower(0.5);
                } else if (robot.drive.getTargetDistanceRemaining() > 500) {
                    robot.drive.setPower(1);
                }

                if (robot.drive.getTargetDistanceRemaining() < 1750) {
                    robot.arm.setArm(Constants.ArmPosition.DOWN);
                }
                if (robot.drive.getTargetDistanceRemaining() < 500) {
                    robot.arm.setClaw(Constants.ServoPosition.OPEN);
                }
            }
            @Override
            public void end() {}
            @Override
            public boolean isFinished() {
                return !robot.drive.isBusy() || robot.drive.getTargetDistanceRemaining() < 15;
            }
        });
    }
    private void addMoveToGoal() {
        steps.add(new Step("Moving to Goal") {
            @Override
            public void start() {
                robot.sensors.resetGyroHeadingToInitial();
                destinationHeading = 180;
                xRuntime = -1;
                yRuntime = -1;
                zRuntime = -1;
                centeredZ = false;
                centeredY = false;
                movingY = false;
            }
            @Override
            public void whileRunning() {
                if (!centeredZ) {
                    // z movement
                    currentHeading = robot.sensors.getGyroHeading360();
                    if (currentHeading > 180 && currentHeading > destinationHeading + 180) {
                        zErr = 360 - currentHeading + destinationHeading;
                    } else {
                        zErr = (destinationHeading - currentHeading + 180) % 360 - 180;
                    }
                    if (Math.abs(zErr) <= 1) {
                        z = 0;
                        if (zRuntime == -1) {
                            zRuntime = currentRuntime;
                        }
                    } else {
                        zRuntime = -1;
                        z = Math.copySign(Math.max(Math.abs(zErr / 180) * WHEEL_SPEED, 0.15), -zErr);
                    }
                    // set the motor speed
                    robot.drive.setInput(x, y, z);
                } else if (!centeredY) {
                    if (robot.sensors.getColor() < 0.8) {
                        y = 0.3;
                        yRuntime = -1;
                    } else {
                        y = 0;
                        if (yRuntime == -1) {
                            yRuntime = currentRuntime;
                        }
                    }
                    robot.drive.setInput(0, 0.3, 0);
                } else if (movingY && !robot.drive.isBusy()) {
                    movingY = false;
                } else {
                    red = robot.camera.getRed();
                    if (red.isValid()) {
                        xErr = red.getCenter().x - 9.5;
                        if (Math.abs(xErr) <= 0.5) {
                            x = 0;
                            if (xRuntime == -1) {
                                xRuntime = currentRuntime;
                            }
                        } else {
                            xRuntime = -1;
//                        x = Math.copySign(Math.max(Math.abs(xErr / 50) * WHEEL_SPEED, 0.15), xErr);
                            x = Math.abs(xErr) > 12 ? Math.copySign(0.5, xErr) : Math.copySign(0.15, xErr);
                        }
                    }
                }

                if (!centeredZ &&  currentRuntime > zRuntime + 1) {
                    centeredZ = true;
                }
                if (!centeredY &&  currentRuntime > yRuntime + 1) {
                    centeredY = true;
                    movingY = true;
                    robot.drive.setTargetPositionRelative(0, -3, 0.5);
                }
            }
            @Override
            public void end() {
                robot.drive.setInput(0, 0, 0);
            }
            @Override
            public boolean isFinished() {
                return currentRuntime > xRuntime + 1 && xRuntime != -1;
            }
        });
    };
}