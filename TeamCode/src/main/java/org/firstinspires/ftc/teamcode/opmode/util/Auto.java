package org.firstinspires.ftc.teamcode.opmode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.util.enums.Alliance;
import org.firstinspires.ftc.teamcode.util.enums.StarterStack;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Locale;

import static org.firstinspires.ftc.teamcode.util.Configurables.AUTO_AIM_A;
import static org.firstinspires.ftc.teamcode.util.Configurables.AUTO_AIM_EXP;
import static org.firstinspires.ftc.teamcode.util.Configurables.AUTO_AIM_H;
import static org.firstinspires.ftc.teamcode.util.Configurables.AUTO_AIM_MAX_ERROR;
import static org.firstinspires.ftc.teamcode.util.Configurables.AUTO_AIM_WAIT;
import static org.firstinspires.ftc.teamcode.util.Configurables.PUSHER_DELAY;
import static org.firstinspires.ftc.teamcode.util.Configurables.SHOOTER_AUTO_AIM_OFFSET_X;
import static org.firstinspires.ftc.teamcode.util.Configurables.SHOOTER_GOAL_POWER;
import static org.firstinspires.ftc.teamcode.util.Configurables.SHOOTER_POWERSHOT_POWER;
import static org.firstinspires.ftc.teamcode.util.enums.Position.CLOSED;
import static org.firstinspires.ftc.teamcode.util.enums.Position.OPEN;
import static org.firstinspires.ftc.teamcode.util.enums.Position.UP;

// Abstract Auto Program
public abstract class Auto extends LinearOpMode {
    public Alliance alliance;
    public Robot robot;
    private StarterStack stack;
    private ArrayList<Step> steps;
    private double currentRuntime;

    public void setAlliance() {
        this.alliance = Alliance.RED;
    }

    @Override
    public void runOpMode() {
        // init
        telemetry.addLine("Initializing Robot...");
        telemetry.update();
        setAlliance();
        robot = new Robot(hardwareMap);
        robot.shooter.setPusher(OPEN);
        robot.intake.setShield(UP);
        robot.camera.initStackCamera();
        while (robot.camera.getFrameCount() < 1) {
            idle();
        }

        // wait for start
        while (!(isStarted() || isStopRequested())) {
            stack = robot.camera.checkStack();
            telemetry.addLine("Initialized");
            telemetry.addLine(robot.camera.getTelemetry());
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

        // run steps
        steps = new ArrayList<>();
        buildSteps(stack);
        int stepNumber = 0;
        Step step = steps.get(stepNumber);
        double stepTimeout = step.getTimeout() != -1 ? currentRuntime + step.getTimeout() : Double.MAX_VALUE;
        step.start();
        while(opModeIsActive() && !isStopRequested()) {
            currentRuntime = getRuntime();

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

            step.whileRunning();
            PoseStorage.currentPose = robot.drive.getPoseEstimate();

            // while the step is running display telemetry
            telemetry.addLine(String.format(Locale.US, "Runtime: %.0f", currentRuntime));
            telemetry.addLine("Step "+(stepNumber+1)+" of "+steps.size()+", "+step.getTelemetry()+"\n");
            telemetry.addLine(robot.getTelemetry());
            telemetry.update();
        }
    }

    public void buildSteps(StarterStack stack) {
        steps = new ArrayList<>();
        delay(5);
    }

    public void delay(double timeout) {
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

    public void setIntake(double timeout, double intakePower) {
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

    public void followTrajectory(Trajectory trajectory) {
        steps.add(new Step("Following a trajectory") {
            @Override
            public void start() {
                robot.drive.followTrajectoryAsync(trajectory);
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

    public void shootRings(double timeout, boolean shootPowershots, int rings) {
        steps.add(new Step("Shooting rings", timeout) {
            @Override
            public void start() {
                ringsToFire = rings;
                powershotsKnockedDown = !shootPowershots;
                if (shootPowershots) {
                    robot.shooter.setShooter(SHOOTER_POWERSHOT_POWER);
                } else {
                    robot.shooter.setShooter(SHOOTER_GOAL_POWER);
                }
                shootingDelay = -1;
            }
            @Override
            public void whileRunning() {
                if (!firing) {
                    // determine offset of target
                    if (!powershotsKnockedDown) {
                        if (alliance == Alliance.RED) {
                            powershot = robot.camera.getRedPowershots().getLeftMost();
                        } else {
                            powershot = robot.camera.getBluePowershots().getLeftMost();
                        }
                        if (powershot.isValid()) {
                            targetPos = powershot.getCenter().x + SHOOTER_AUTO_AIM_OFFSET_X;
                        } else {
                            powershotsKnockedDown = true;
                            robot.shooter.setShooter(SHOOTER_GOAL_POWER);
                        }
                    } else {
                        if (alliance == Alliance.RED) {
                            goal = robot.camera.getRed();
                        } else {
                            goal = robot.camera.getBlue();
                        }
                        if (goal.isValid()) {
                            targetPos = goal.getCenter().x + SHOOTER_AUTO_AIM_OFFSET_X;
                        }
                    }
                    // either start firing or move towards target
                    if (Math.abs(targetPos) <= AUTO_AIM_H) {
                        if (shootingDelay == -1) {
                            shootingDelay = currentRuntime;
                        }
                    } else {
                        shootingDelay = -1;
                    }
                    if (shootingDelay != -1 && currentRuntime >= shootingDelay + AUTO_AIM_WAIT) {
                        z = 0;
                        firing = true;
                        robot.shooter.setPusher(CLOSED);
                        zig = true;
                        zag = false;
                        zigTime = getRuntime();
                    } else {
                        double x2 = Math.abs(targetPos);
                        double power = AUTO_AIM_A * Math.pow((x2 - AUTO_AIM_H), 1/AUTO_AIM_EXP);
                        if (x2 < AUTO_AIM_H || x2 > AUTO_AIM_MAX_ERROR) {
                            z = 0;
                        } else {
                            z = Math.copySign(power, -targetPos);
                        }
                    }
                } else {
                    // wait while servo is moving
                    if (zig && getRuntime() > zigTime + PUSHER_DELAY) {
                        robot.shooter.setPusher(OPEN);
                        zig = false;
                        zag = true;
                        zagTime = getRuntime();
                    } else if (zag && getRuntime() > zagTime + (powershotsKnockedDown ? PUSHER_DELAY : 1)) {
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

    public void stopTargetingCamera() {
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

    public MinVelocityConstraint getVelocityConstraint(double constraint) {
        return new MinVelocityConstraint(
                Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(constraint, DriveConstants.TRACK_WIDTH)
                )
        );
    }

    public ProfileAccelerationConstraint getAccelerationConstraint() {
        return new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL);
    }
}
