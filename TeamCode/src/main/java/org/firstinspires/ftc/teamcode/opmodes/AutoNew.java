package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Locale;

import static org.firstinspires.ftc.teamcode.Constants.ARM_ALMOST_DOWN_POS;
import static org.firstinspires.ftc.teamcode.Constants.ARM_DEFAULT_POS;
import static org.firstinspires.ftc.teamcode.Constants.ARM_DOWN_POS;
import static org.firstinspires.ftc.teamcode.Constants.AUTO_AIM_OFFSET_X;
import static org.firstinspires.ftc.teamcode.Constants.CLAW_WAIT;
import static org.firstinspires.ftc.teamcode.Constants.POWERSHOT_SHOOTER_POWER;
import static org.firstinspires.ftc.teamcode.Constants.SHOOTER_POWER;

@Autonomous(name = "New Autonomous", group = "Testing")
public class AutoNew extends LinearOpMode {
    private Robot robot;
    private Constants.StarterStack stack;
    private ArrayList<Step> steps;
    private double currentRuntime;

    private Pose2d startPose;
    private Trajectory dropOffFirstWobbleGoal;
    private Trajectory driveToGoal;
    private Trajectory driveToSecondWobbleGoal;
    private Trajectory pickUpSecondWobbleGoal;
    private Trajectory dropOffSecondWobbleGoal;
    private Trajectory park;

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
        startPose = new Pose2d(-63.5, -55.75, Math.toRadians(180));
        robot.drive.setPoseEstimate(startPose);

        dropOffFirstWobbleGoal = robot.drive.trajectoryBuilder(startPose)
                .addTemporalMarker(0, () -> robot.shooter.setPusher(Constants.ServoPosition.OPEN))
                .lineTo(new Vector2d(0, -55.75))
                .build();
        driveToGoal = robot.drive.trajectoryBuilder(dropOffFirstWobbleGoal.end().plus(new Pose2d(0, 0, Math.toRadians(-45))))
                .addTemporalMarker(0, () -> robot.shooter.setShooter(SHOOTER_POWER))
                .addTemporalMarker(0, () -> robot.arm.setArm(ARM_DOWN_POS))
                .lineToLinearHeading(new Pose2d(-5, -40, Math.toRadians(0)))
                .build();
        driveToSecondWobbleGoal = robot.drive.trajectoryBuilder(driveToGoal.end())
                .addTemporalMarker(0, () -> robot.shooter.setShooter(0))
                .addTemporalMarker(0, () -> robot.arm.setClaw(Constants.ServoPosition.OPEN))
                .lineToLinearHeading(new Pose2d(-30, -38, Math.toRadians(0)))
                .build();
        pickUpSecondWobbleGoal = robot.drive.trajectoryBuilder(driveToSecondWobbleGoal.end())
                .addTemporalMarker(0.5, () -> robot.arm.setClaw(Constants.ServoPosition.CLOSED))
                .back(5, new MinVelocityConstraint(
                        Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(5, DriveConstants.TRACK_WIDTH)
                        )
                    ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        dropOffSecondWobbleGoal = robot.drive.trajectoryBuilder(pickUpSecondWobbleGoal.end())
                .addTemporalMarker(0, () -> robot.arm.setArm(ARM_ALMOST_DOWN_POS))
                .addTemporalMarker(3, () -> robot.arm.setClaw(Constants.ServoPosition.OPEN))
                .lineToLinearHeading(new Pose2d(-15, -55, Math.toRadians(180)))
                .build();
        park = robot.drive.trajectoryBuilder(dropOffSecondWobbleGoal.end())
                .addTemporalMarker(0, () -> robot.arm.setArm(ARM_DEFAULT_POS))
                .addTemporalMarker(0.4, () -> robot.arm.setClaw(Constants.ServoPosition.CLOSED))
                .lineToLinearHeading(new Pose2d(0, -32, Math.toRadians(180)))
                .build();

        steps = new ArrayList<>();
        switch(stack) {
            case NONE:
                followTrajectory(dropOffFirstWobbleGoal);
                turn(-45);
                setIntake(0.5, 0.5);
                setIntake(0, 0);
                followTrajectory(driveToGoal);
                setPusher(0.2, Constants.ServoPosition.CLOSED);
                setPusher(0.2, Constants.ServoPosition.OPEN);
                setPusher(0.2, Constants.ServoPosition.CLOSED);
                setPusher(0.2, Constants.ServoPosition.OPEN);
                setPusher(0.2, Constants.ServoPosition.CLOSED);
                setPusher(0, Constants.ServoPosition.OPEN);
//                shootRings(5, false, 3);
                followTrajectory(driveToSecondWobbleGoal);
                followTrajectory(pickUpSecondWobbleGoal);
                followTrajectory(dropOffSecondWobbleGoal);
                followTrajectory(park);
                stopTargetingCamera();
                break;
            case SINGLE:
                break;
            case QUAD:
        }
        stopTargetingCamera();
    }

    private void setPusher(double timeout, final Constants.ServoPosition position) {
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
    private void setShooter(double timeout, double shooterPower) {
        steps.add(new Step("Setting shooter " + shooterPower, timeout) {
            @Override
            public void start() {
                robot.intake.setIntake(shooterPower);
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
                    if (Math.abs(px) <= 0.3) {
                        z = 0;
                        firing = true;
                        robot.shooter.setPusher(Constants.ServoPosition.CLOSED);
                        zig = true;
                        zag = false;
                        zigTime = getRuntime();
                    } else {
                        z = Math.copySign(Math.max(Math.abs((px / 50) * 0.7), 0.05), px);
                    }
                } else {
                    if (zig && getRuntime() > zigTime + CLAW_WAIT) {
                        robot.shooter.setPusher(Constants.ServoPosition.OPEN);
                        zig = false;
                        zag = true;
                        zagTime = getRuntime();
                    } else if (zag && getRuntime() > zagTime + CLAW_WAIT) {
                        firing = false;
                        ringsFired++;
                    }
                }
                robot.drive.setWeightedDrivePower(new Pose2d(0, 0, -z));
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
