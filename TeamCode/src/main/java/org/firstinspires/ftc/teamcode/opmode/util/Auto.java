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
import org.firstinspires.ftc.teamcode.util.enums.Position;
import org.firstinspires.ftc.teamcode.util.enums.StarterStack;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Locale;

import static org.firstinspires.ftc.teamcode.hardware.Lights.BLUE_AIMED_AND_READY;
import static org.firstinspires.ftc.teamcode.hardware.Lights.BLUE_AIMING;
import static org.firstinspires.ftc.teamcode.hardware.Lights.BLUE_NORMAL;
import static org.firstinspires.ftc.teamcode.hardware.Lights.RED_AIMED_AND_READY;
import static org.firstinspires.ftc.teamcode.hardware.Lights.RED_AIMING;
import static org.firstinspires.ftc.teamcode.hardware.Lights.RED_NORMAL;
import static org.firstinspires.ftc.teamcode.util.Configurables.AUTO_AIM_ACCEPTABLE_ERROR;
import static org.firstinspires.ftc.teamcode.util.Configurables.AUTO_AIM_MIN_POWER;
import static org.firstinspires.ftc.teamcode.util.Configurables.AUTO_AIM_OFFSET_X;
import static org.firstinspires.ftc.teamcode.util.Configurables.AUTO_AIM_PID;
import static org.firstinspires.ftc.teamcode.util.Configurables.AUTO_AIM_WAIT;
import static org.firstinspires.ftc.teamcode.util.Configurables.PUSHER_DELAY;
import static org.firstinspires.ftc.teamcode.util.Configurables.SHOOTER_GOAL_POWER;
import static org.firstinspires.ftc.teamcode.util.Configurables.SHOOTER_POWERSHOT_POWER;
import static org.firstinspires.ftc.teamcode.util.enums.Position.CLOSED;
import static org.firstinspires.ftc.teamcode.util.enums.Position.OPEN;
import static org.firstinspires.ftc.teamcode.util.enums.Position.UP;

// Abstract Auto Program
public abstract class Auto extends LinearOpMode {
    public Alliance alliance;
    public boolean checkForStarterStack;
    public Robot robot;
    private ArrayList<Step> steps;
    private double stepTimeout;
    private double currentRuntime;
    private StarterStack stack = StarterStack.NONE;

    PIDFController controller;

    @Override
    public void runOpMode() {
        // init
        telemetry.addLine("Initializing Robot...");
        telemetry.update();

        robot = new Robot(hardwareMap);
        robot.shooter.setPusher(OPEN);
        robot.intake.setShield(UP);

        controller = new PIDFController(0, 0, 0, 0);

        setAlliance();
        setCamera();

        if (alliance == Alliance.RED) {
            robot.lights.setPattern(RED_NORMAL);
        } else if (alliance == Alliance.BLUE) {
            robot.lights.setPattern(BLUE_NORMAL);
        }

        if (checkForStarterStack) {
            // start stack camera
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
        } else {
            // start targeting camera
            robot.camera.initTargetingCamera();
            while(robot.camera.getFrameCount() < 1) {
                idle();
            }

            // wait for start
            while (!(isStarted() || isStopRequested())) {
                telemetry.addLine("Initialized");
                telemetry.update();
            }
            if (isStopRequested()) return;
            resetStartTime();
        }

        // run steps
        steps = new ArrayList<>();
        buildSteps(stack);
        int stepNumber = 0;
        Step step = steps.get(stepNumber);
        stepTimeout = step.getTimeout() != -1 ? currentRuntime + step.getTimeout() : Double.MAX_VALUE;
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

    public void setAlliance() {
        alliance = Alliance.RED;
    }

    public void setCamera() {
        checkForStarterStack = true;
    }

    public void buildSteps(StarterStack stack) {
        steps = new ArrayList<>();
        delay(5);
        stopTargetingCamera();
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

    public void setClaw(double timeout, Position position) {
        steps.add(new Step("Setting arm to " + position, timeout) {
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

    public void setArm(double timeout, Position position) {
        steps.add(new Step("Setting arm to " + position, timeout) {
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
                return false;
            }
        });
    }

    public void turn(double degrees) {
        steps.add(new Step("Following a trajectory") {
            @Override
            public void start() {
                robot.drive.turn(degrees);
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
                            targetPos = powershot.getCenter().x + AUTO_AIM_OFFSET_X;
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
                            targetPos = goal.getCenter().x + AUTO_AIM_OFFSET_X;
                        }
                    }
                    // either start firing or move towards target
                    if (Math.abs(targetPos) <= AUTO_AIM_ACCEPTABLE_ERROR || currentRuntime >= stepTimeout - 2) {
                        if (shootingDelay == -1) {
                            shootingDelay = currentRuntime;
                        }
                    } else {
                        shootingDelay = -1;
                    }
                    if (shootingDelay != -1 && currentRuntime >= shootingDelay + AUTO_AIM_WAIT) {
                        if (alliance == Alliance.RED) {
                            robot.lights.setPattern(RED_AIMED_AND_READY);
                        } else if (alliance == Alliance.BLUE) {
                            robot.lights.setPattern(BLUE_AIMED_AND_READY);
                        }
                        z = 0;
                        firing = true;
                        robot.shooter.setPusher(CLOSED);
                        zig = true;
                        zag = false;
                        zigTime = currentRuntime;
                    } else {
                        if (alliance == Alliance.RED) {
                            robot.lights.setPattern(RED_AIMING);
                        } else if (alliance == Alliance.BLUE) {
                            robot.lights.setPattern(BLUE_AIMING);
                        }
                        controller.setPIDF(AUTO_AIM_PID.p, AUTO_AIM_PID.i, AUTO_AIM_PID.d, AUTO_AIM_PID.f);
                        controller.setTolerance(0.1);
                        double output = -controller.calculate(0, targetPos);
                        z = Math.abs(controller.getPositionError()) <= AUTO_AIM_ACCEPTABLE_ERROR ? 0 : Math.copySign(Math.max(AUTO_AIM_MIN_POWER, Math.abs(output)), output);
                    }
                } else {
                    // wait while servo is moving
                    if (zig && currentRuntime > zigTime + PUSHER_DELAY) {
                        robot.shooter.setPusher(OPEN);
                        zig = false;
                        zag = true;
                        zagTime = currentRuntime;
                    } else if (zag && currentRuntime > zagTime + (powershotsKnockedDown ? PUSHER_DELAY : 1)) {
                        firing = false;
                        ringsFired++;
                    }
                }
                robot.drive.setWeightedDrivePower(new Pose2d(0, 0, z));
                robot.drive.update();
                setTelemetry("\nz: "+z+"\nerror: "+targetPos);
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
