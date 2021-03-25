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
import static org.firstinspires.ftc.teamcode.Constants.INTAKE_SPEED;
import static org.firstinspires.ftc.teamcode.Constants.PUSHER_DELAY;
import static org.firstinspires.ftc.teamcode.Constants.SHOOTER_AUTO_AIM_OFFSET_X;
import static org.firstinspires.ftc.teamcode.Constants.SHOOTER_GOAL_POWER;
import static org.firstinspires.ftc.teamcode.Constants.SHOOTER_POWERSHOT_POWER;

// Main Autonomous Program
@Autonomous(name = "Autonomous", group = "Competition", preselectTeleOp = "TeleOp")
public class Auto extends LinearOpMode {
    private Robot robot;
    private Constants.StarterStack stack;
    private ArrayList<Step> steps;
    private double currentRuntime;

    private Pose2d startPose;

    private Trajectory noneDropOffFirstWobbleGoal;
    private Trajectory noneDriveToPowershots;
    private Trajectory noneDriveToSecondWobbleGoal;
    private Trajectory nonePickUpSecondWobbleGoal;
    private Trajectory noneDropOffSecondWobbleGoal;
    private Trajectory nonePark;

    private Trajectory singleDropOffFirstWobbleGoal;
    private Trajectory singleDriveToPowershots;
    private Trajectory singleDriveToRing;
    private Trajectory singlePickUpSecondWobbleGoal;
    private Trajectory singleDriveToGoal;
    private Trajectory singleDropOffSecondWobbleGoal;
    private Trajectory singlePark;

    private Trajectory quadDropOffFirstWobbleGoal;
    private Trajectory quadDriveToGoal;
    private Trajectory quadDriveToRings;
    private Trajectory quadPickUpSomeRings;
    private Trajectory quadDriveToGoalAgain;
    private Trajectory quadPickUpSecondWobbleGoal;
    private Trajectory quadDriveToGoalOnceMore;
    private Trajectory quadDropOffSecondWobbleGoal;
    private Trajectory quadPark;

    @Override
    public void runOpMode() {
        // initialize robot
        telemetry.addLine("Initializing Robot...");
        telemetry.update();
        robot = new Robot(hardwareMap);
        robot.camera.initStackCamera();
        robot.shooter.setPusher(Constants.ServoPosition.OPEN);
        robot.intake.setShield(Constants.ServoPosition.OPEN);
        initializeTrajectories();
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

    // Initialize trajectories
    private void initializeTrajectories() {
        startPose = new Pose2d(-63.5, -55.75, Math.toRadians(180));

        Pose2d powershots = new Pose2d(-6, -6, Math.toRadians(0));
        Pose2d goal = new Pose2d(-6, -44, Math.toRadians(0));
        Pose2d park = new Pose2d(6, -24, Math.toRadians(180));

        // no rings
        noneDropOffFirstWobbleGoal = robot.drive.trajectoryBuilder(startPose)
                .addTemporalMarker(1, () -> robot.shooter.setShooter(SHOOTER_POWERSHOT_POWER))
                .lineToLinearHeading(new Pose2d(0, -55.75, Math.toRadians(135)))
                .build();
        noneDriveToPowershots = robot.drive.trajectoryBuilder(noneDropOffFirstWobbleGoal.end())
                .addTemporalMarker(0, () -> robot.arm.setArm(ARM_DOWN_POS))
                .lineToLinearHeading(powershots)
                .build();
        noneDriveToSecondWobbleGoal = robot.drive.trajectoryBuilder(noneDriveToPowershots.end())
                .addTemporalMarker(0, () -> robot.shooter.setShooter(0))
                .addTemporalMarker(0, () -> robot.arm.setClaw(Constants.ServoPosition.OPEN))
                .lineToLinearHeading(new Pose2d(-32, -34.5, Math.toRadians(0)))
                .build();
        nonePickUpSecondWobbleGoal = robot.drive.trajectoryBuilder(noneDriveToSecondWobbleGoal.end())
                .addTemporalMarker(0.5, () -> robot.arm.setClaw(Constants.ServoPosition.CLOSED))
                .back(5, new MinVelocityConstraint(
                        Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(5, DriveConstants.TRACK_WIDTH)
                        )
                ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        noneDropOffSecondWobbleGoal = robot.drive.trajectoryBuilder(nonePickUpSecondWobbleGoal.end())
                .addTemporalMarker(0, () -> robot.arm.setArm(ARM_ALMOST_DOWN_POS))
                .addTemporalMarker(3.3, () -> robot.arm.setClaw(Constants.ServoPosition.OPEN))
                .lineToLinearHeading(new Pose2d(-2, -41, Math.toRadians(135)))
                .build();
        nonePark = robot.drive.trajectoryBuilder(noneDropOffSecondWobbleGoal.end())
                .addTemporalMarker(0, () -> robot.arm.setArm(ARM_DEFAULT_POS))
                .addTemporalMarker(0.4, () -> robot.arm.setClaw(Constants.ServoPosition.CLOSED))
                .lineToLinearHeading(park)
                .build();

        // one ring
        singleDropOffFirstWobbleGoal = robot.drive.trajectoryBuilder(startPose)
                .addTemporalMarker(1.5, () -> robot.shooter.setShooter(SHOOTER_POWERSHOT_POWER))
                .back(26)
                .splineToConstantHeading(new Vector2d(19, -41), Math.toRadians(0))
                .build();
        singleDriveToPowershots = robot.drive.trajectoryBuilder(singleDropOffFirstWobbleGoal.end())
                .addTemporalMarker(0, () -> robot.arm.setArm(ARM_DOWN_POS))
                .splineTo(new Vector2d(powershots.getX(), powershots.getY()), powershots.getHeading())
                .build();
        singleDriveToRing = robot.drive.trajectoryBuilder(singleDriveToPowershots.end())
                .addTemporalMarker(0, () -> robot.arm.setClaw(Constants.ServoPosition.OPEN))
                .lineToLinearHeading(new Pose2d(-8, -39, Math.toRadians(0)))
                .build();
        singlePickUpSecondWobbleGoal = robot.drive.trajectoryBuilder(singleDriveToRing.end())
                .addTemporalMarker(0, () -> robot.shooter.setShooter(SHOOTER_GOAL_POWER))
                .addTemporalMarker(0, () -> robot.intake.setIntake(INTAKE_SPEED))
                .addTemporalMarker(1, () -> robot.arm.setClaw(Constants.ServoPosition.CLOSED))
                .lineToLinearHeading(new Pose2d(-36,-34.5, Math.toRadians(0)))
                .build();
        singleDriveToGoal = robot.drive.trajectoryBuilder(singlePickUpSecondWobbleGoal.end())
                .addTemporalMarker(0, () -> robot.arm.setArm(ARM_ALMOST_DOWN_POS))
                .lineToLinearHeading(goal)
                .build();
        singleDropOffSecondWobbleGoal = robot.drive.trajectoryBuilder(singleDriveToGoal.end())
                .addTemporalMarker(0, () -> robot.shooter.setShooter(0))
                .addTemporalMarker(0, () -> robot.intake.setIntake(0))
                .addTemporalMarker(3.5, () -> robot.arm.setClaw(Constants.ServoPosition.OPEN))
                .lineToLinearHeading(new Pose2d(10, -26, Math.toRadians(180)))
                .build();
        singlePark = robot.drive.trajectoryBuilder(singleDropOffSecondWobbleGoal.end())
                .addTemporalMarker(0, () -> robot.arm.setArm(ARM_DEFAULT_POS))
                .addTemporalMarker(0.4, () -> robot.arm.setClaw(Constants.ServoPosition.CLOSED))
                .lineToLinearHeading(park)
                .build();

        // four rings
        quadDropOffFirstWobbleGoal = robot.drive.trajectoryBuilder(startPose)
                .addTemporalMarker(2, () -> robot.shooter.setShooter(SHOOTER_GOAL_POWER))
                .lineToLinearHeading(new Pose2d(44, -55.75, Math.toRadians(135)))
                .build();
        quadDriveToGoal = robot.drive.trajectoryBuilder(quadDropOffFirstWobbleGoal.end())
                .addTemporalMarker(0, () -> robot.arm.setArm(ARM_DOWN_POS))
                .addTemporalMarker(1, () -> robot.intake.setShield(Constants.ServoPosition.CLOSED))
                .lineToLinearHeading(goal)
                .build();
        quadDriveToRings = robot.drive.trajectoryBuilder(quadDriveToGoal.end())
                .addTemporalMarker(0, () -> robot.arm.setClaw(Constants.ServoPosition.OPEN))
                .lineToLinearHeading(new Pose2d(-8, -38, Math.toRadians(0)))//x:-18
                .build();
        quadPickUpSomeRings = robot.drive.trajectoryBuilder(quadDriveToRings.end())
                .addTemporalMarker(0, () -> robot.shooter.setShooter(SHOOTER_GOAL_POWER))
                .addTemporalMarker(0, () -> robot.intake.setIntake(INTAKE_SPEED))
                .back(13, new MinVelocityConstraint(
                        Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                        )
                ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        quadDriveToGoalAgain = robot.drive.trajectoryBuilder(quadPickUpSomeRings.end())
                .lineToLinearHeading(new Pose2d(goal.getX(), -38, goal.getHeading()))
                .build();
        quadPickUpSecondWobbleGoal = robot.drive.trajectoryBuilder(quadDriveToGoalAgain.end())
                .addTemporalMarker(2.75, () -> robot.arm.setClaw(Constants.ServoPosition.CLOSED))
                .lineToLinearHeading(new Pose2d(-39,-35, Math.toRadians(0)), new MinVelocityConstraint(
                        Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                        )
                ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        quadDriveToGoalOnceMore = robot.drive.trajectoryBuilder(quadPickUpSecondWobbleGoal.end())
                .addTemporalMarker(0, () -> robot.arm.setArm(ARM_ALMOST_DOWN_POS))
                .lineToLinearHeading(goal)
                .build();
        quadDropOffSecondWobbleGoal = robot.drive.trajectoryBuilder(quadDriveToGoalOnceMore.end())
                .addTemporalMarker(0, () -> robot.shooter.setShooter(0))
                .addTemporalMarker(0, () -> robot.intake.setIntake(0))
                .addTemporalMarker(0, () -> robot.intake.setShield(Constants.ServoPosition.OPEN))
                .addTemporalMarker(3, () -> robot.arm.setClaw(Constants.ServoPosition.OPEN))
                .lineToLinearHeading(new Pose2d(32, -53, Math.toRadians(180)))
                .build();
        quadPark = robot.drive.trajectoryBuilder(quadDropOffSecondWobbleGoal.end())
                .addTemporalMarker(0, () -> robot.arm.setArm(ARM_DEFAULT_POS))
                .addTemporalMarker(0.4, () -> robot.arm.setClaw(Constants.ServoPosition.CLOSED))
                .lineToLinearHeading(park)
                .build();
    }

    // Load up all of the steps for the autonomous
    private void initializeSteps(Constants.StarterStack stack) {
        robot.drive.setPoseEstimate(startPose);
        steps = new ArrayList<>();
        switch(stack) {
            case NONE:
                followTrajectory(noneDropOffFirstWobbleGoal);
                setIntake(0.5, 0.5);
                setIntake(0, 0);
                followTrajectory(noneDriveToPowershots);
                shootRings(8, true, 3);
                followTrajectory(noneDriveToSecondWobbleGoal);
                followTrajectory(nonePickUpSecondWobbleGoal);
                followTrajectory(noneDropOffSecondWobbleGoal);
                followTrajectory(nonePark);
                break;
            case SINGLE:
                followTrajectory(singleDropOffFirstWobbleGoal);
                setIntake(0.5, 0.5);
                setIntake(0, 0);
                followTrajectory(singleDriveToPowershots);
                shootRings(8, true, 3);
                followTrajectory(singleDriveToRing);
                followTrajectory(singlePickUpSecondWobbleGoal);
                followTrajectory(singleDriveToGoal);
                shootRings(3, false, 1);
                followTrajectory(singleDropOffSecondWobbleGoal);
                followTrajectory(singlePark);
                break;
            case QUAD:
                followTrajectory(quadDropOffFirstWobbleGoal);
                setIntake(0.5, 0.5);
                setIntake(0, 0);
                followTrajectory(quadDriveToGoal);
                shootRings(8, false, 3);
                followTrajectory(quadDriveToRings);
                followTrajectory(quadPickUpSomeRings);
                followTrajectory(quadDriveToGoalAgain);
                shootRings(3, false, 1);
                followTrajectory(quadPickUpSecondWobbleGoal);
                followTrajectory(quadDriveToGoalOnceMore);
                shootRings(8, false, 3);
                followTrajectory(quadDropOffSecondWobbleGoal);
                followTrajectory(quadPark);
        }
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
                powershotsKnockedDown = !shootPowershots;
            }
            @Override
            public void whileRunning() {
                if (!firing) {
                    // determine offset of target
                    if (!powershotsKnockedDown) {
                        powershot = robot.camera.getPowershots().getLeftMost();
                        if (powershot.isValid()) {
                            targetPos = powershot.getCenter().x + SHOOTER_AUTO_AIM_OFFSET_X;
                        } else {
                            powershotsKnockedDown = true;
                            robot.shooter.setShooter(SHOOTER_GOAL_POWER);
                        }
                    } else {
                        red = robot.camera.getRed();
                        if (red.isValid()) {
                            targetPos = red.getCenter().x + SHOOTER_AUTO_AIM_OFFSET_X;
                        }
                    }
                    // either start firing or move towards target
                    if (Math.abs(targetPos) <= 0.5) {
                        z = 0;
                        firing = true;
                        robot.shooter.setPusher(Constants.ServoPosition.CLOSED);
                        zig = true;
                        zag = false;
                        zigTime = getRuntime();
                    } else {
                        z = Math.copySign(Math.max(Math.abs((targetPos / 50) * 0.8), 0.15), -targetPos);
                    }
                } else {
                    // wait while servo is moving
                    if (zig && getRuntime() > zigTime + PUSHER_DELAY) {
                        robot.shooter.setPusher(Constants.ServoPosition.OPEN);
                        zig = false;
                        zag = true;
                        zagTime = getRuntime();
                    } else if (zag && getRuntime() > zagTime + (powershotsKnockedDown ? 0.2 : 1)) {
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
