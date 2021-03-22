package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.ArrayList;
import java.util.Locale;

import static org.firstinspires.ftc.teamcode.Constants.AUTO_AIM_OFFSET_X;
import static org.firstinspires.ftc.teamcode.Constants.SHOOTER_POWER;

@Autonomous(name = "Powershot Test", group = "Competition")
public class AutoPowershotTest extends LinearOpMode {
    private Robot robot;
    private Constants.StarterStack stack;
    private ArrayList<Step> steps;
    private double currentRuntime;

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
            telemetry.addLine(String.format(Locale.US, "Stack: %s", stack));
            telemetry.addLine(String.format(Locale.US, "Size: %.4f", robot.camera.getStarterStack().getArea()));
            telemetry.update();
        }
        if (isStopRequested()) return;
        resetStartTime();

        // switch cameras
        robot.camera.stopStackCamera();
        while(robot.camera.getFrameCount() > 0) {
            idle();
        }
        robot.camera.initTargetingCamera();
        while(robot.camera.getFrameCount() < 1) {
            idle();
        }

        // start up the first step
        initializeSteps(stack);
        int stepNumber = 0;
        Step step = steps.get(stepNumber);
        double stepTimeout = step.getTimeout() != -1 ? currentRuntime + step.getTimeout() : Double.MAX_VALUE;
        step.start();

        // run the remaining steps
        while(opModeIsActive() && !isStopRequested()) {
            currentRuntime = getRuntime();

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

    // Load up all of the steps for the autonomous
    private void initializeSteps(Constants.StarterStack stack) {
        steps = new ArrayList<>();
        shootRings(8, true, 3);
        stopTargetingCamera();
    }

    // Functions to add steps
    private void wait(double timeout) {
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
    private void setIntake(double timeout, double intakePower) {
        steps.add(new Step("Setting intake " + intakePower, timeout) {
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
    private void setPusher(double timeout, Constants.ServoPosition position) {
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
    private void followTrajectory(Trajectory trajectory) {
        steps.add(new Step("Following a trajectory") {
            @Override
            public void start() {
                robot.drive.followTrajectoryAsync(trajectory);
                intakeJammedTime = Double.MAX_VALUE;
                checkForJam = currentRuntime;
                lastIntakePosition = robot.intake.getIntakePosition();
                unJamming = false;
            }
            @Override
            public void whileRunning() {
                robot.drive.update();
                // check for jams
                if (Math.abs(robot.intake.getIntakePower()) > 0.1 && currentRuntime > checkForJam + 0.25 && !unJamming) {
                    currentIntakePosition = robot.intake.getIntakePosition();
                    if (Math.abs(currentIntakePosition - lastIntakePosition) < 10) {
                        robot.intake.setIntake(-0.5);
                        lastIntakePosition = currentIntakePosition;
                        intakeJammedTime = currentRuntime;
                        unJamming = true;
                    }
                    checkForJam = currentRuntime;
                }
                if (currentRuntime > intakeJammedTime + 0.5) {
                    robot.intake.setIntake(0.5);
                    intakeJammedTime = Double.MAX_VALUE;
                    unJamming = false;
                }
            }
            @Override
            public void end() {}
            @Override
            public boolean isFinished() {
                return !robot.drive.isBusy();
            }
        });
    }
    private void turn(int degrees) {
        steps.add(new Step("turning "+degrees+" degrees") {
            @Override
            public void start() {
                robot.drive.turnAsync(Math.toRadians(degrees));
            }
            @Override
            public void whileRunning() {
                robot.drive.update();
            }
            @Override
            public void end() {}
            @Override
            public boolean isFinished() {
                return !robot.drive.isBusy();
            }
        });
    }
    private void shootRings(double timeout, boolean shootPowershots, int rings) {
        steps.add(new Step("Shooting rings", timeout) {
            @Override
            public void start() {
                ringsToFire = rings;
                ringsFired = 0;
                firing = false;
                powershotsKnockedDown = !shootPowershots;
                aimedAtTarget = false;
                z = 0;
            }
            @Override
            public void whileRunning() {
                if (!firing) {
                    // determine offset of target
                    double px = 0;
                    if (!powershotsKnockedDown) {
                        powershot = robot.camera.getPowershots().getLeftMost();
                        if (powershot.isValid()) {
                            px = powershot.getCenter().x + AUTO_AIM_OFFSET_X;
                        } else {
                            powershotsKnockedDown = true;
                            robot.shooter.setShooter(SHOOTER_POWER);
                        }
                    } else {
                        red = robot.camera.getRed();
                        if (red.isValid()) {
                            px = red.getCenter().x + AUTO_AIM_OFFSET_X;
                        }
                    }
                    // either start firing or move towards target
                    if (Math.abs(px) <= 0.5) {
                        z = 0;
                        firing = true;
                        robot.shooter.setPusher(Constants.ServoPosition.CLOSED);
                        zig = true;
                        zag = false;
                        zigTime = getRuntime();
                    } else {
                        z = Math.copySign(Math.max(Math.abs((px / 50) * 0.7), 0.1), -px);
                    }
                } else {
                    // wait while servo is moving
                    if (zig && getRuntime() > zigTime + 0.2) {
                        robot.shooter.setPusher(Constants.ServoPosition.OPEN);
                        zig = false;
                        zag = true;
                        zagTime = getRuntime();
                    } else if (zag && getRuntime() > zagTime + (powershotsKnockedDown ? 0.2 : 0.8)) {
                        firing = false;
                        ringsFired++;
                    }
                }
                robot.drive.setWeightedDrivePower(new Pose2d(0, 0, z));
                robot.drive.update();
            }
            @Override
            public void end() {}
            @Override
            public boolean isFinished() {
                return ringsFired >= ringsToFire;
            }
        });
    }
    private void stopTargetingCamera() {
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
                return robot.camera.getFrameCount() == 0;
            }
        });
    }
}
