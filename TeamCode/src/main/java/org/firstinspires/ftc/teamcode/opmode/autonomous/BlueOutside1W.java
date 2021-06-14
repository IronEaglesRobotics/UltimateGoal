package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.util.Auto;
import org.firstinspires.ftc.teamcode.util.enums.Alliance;
import org.firstinspires.ftc.teamcode.util.enums.StarterStack;

import static org.firstinspires.ftc.teamcode.util.Configurables.R_INTAKE_SPEED;
import static org.firstinspires.ftc.teamcode.util.Configurables.R_SHOOTER_GOAL_POWER;
import static org.firstinspires.ftc.teamcode.util.enums.Position.BACK;
import static org.firstinspires.ftc.teamcode.util.enums.Position.CLOSED;
import static org.firstinspires.ftc.teamcode.util.enums.Position.DOWN;
import static org.firstinspires.ftc.teamcode.util.enums.Position.UP;

@Autonomous(name = "Blue Outside 1 Wobble", group = "Competition", preselectTeleOp = "Blue TeleOp")
public class BlueOutside1W extends Auto {
    public static Pose2d START_POSE = new Pose2d(-62.5, 54.85, Math.toRadians(180));

    public static Pose2d NONE_DROP_FIRST_WOBBLE     = new Pose2d(6, 54.85, Math.toRadians(-135));
    public static Pose2d NONE_GOAL                  = new Pose2d(-7, 40, Math.toRadians(0));
    public static Pose2d NONE_DELAY                 = new Pose2d(-35, 38, Math.toRadians(180));
    public static Pose2d NONE_PARK                  = new Pose2d(10, 38, Math.toRadians(180));

    public static Pose2d SINGLE_DROP_FIRST_WOBBLE     = new Pose2d(23, 35, Math.toRadians(0));
    public static Pose2d SINGLE_GOAL                  = new Pose2d(-8, 36, Math.toRadians(0));
    public static Pose2d SINGLE_PICK_UP_RING          = new Pose2d(-25, 36, Math.toRadians(0));
    public static Pose2d SINGLE_GOAL_2                = new Pose2d(-8, 36, Math.toRadians(0));
    public static Pose2d SINGLE_DELAY                 = new Pose2d(-35, 43, Math.toRadians(180));
    public static Pose2d SINGLE_PARK                  = new Pose2d(10, 43, Math.toRadians(180));

    public static Pose2d QUAD_DROP_FIRST_WOBBLE       = new Pose2d(55, 54.85, Math.toRadians(-135));
    public static Pose2d QUAD_GOAL                    = new Pose2d(-7, 39,  Math.toRadians(0));
    public static Pose2d QUAD_PICK_UP_RING            = new Pose2d(-20, 39, Math.toRadians(0));
    public static Pose2d QUAD_GOAL_2                  = new Pose2d(-9, 39, Math.toRadians(0));
    public static Pose2d QUAD_RINGS                   = new Pose2d(-37, 39, Math.toRadians(0));
    public static Pose2d QUAD_GOAL_3                  = new Pose2d(-7, 38, Math.toRadians(0));
    public static Pose2d QUAD_PARK                    = new Pose2d(14, 50, Math.toRadians(180));

    @Override
    public void setAlliance() {
        this.alliance = Alliance.BLUE;
    }

    @Override
    public void setCamera() {
        this.checkForStarterStack = true;
    }

    @Override
    public void buildSteps(StarterStack stack) {
        robot.drive.setPoseEstimate(START_POSE);

        Trajectory noneDropFirstWobble = robot.drive.trajectoryBuilder(START_POSE)
                .addTemporalMarker(0.5, () -> robot.shooter.setShooter(R_SHOOTER_GOAL_POWER))
                .lineToLinearHeading(NONE_DROP_FIRST_WOBBLE)
                .build();
        Trajectory noneGoal = robot.drive.trajectoryBuilder(noneDropFirstWobble.end())
                .addTemporalMarker(0, () -> robot.arm.setArm(DOWN))
                .addTemporalMarker(1, () -> robot.arm.setClaw(CLOSED))
                .lineToLinearHeading(NONE_GOAL)
                .build();
        Trajectory noneDelay = robot.drive.trajectoryBuilder(noneGoal.end())
                .addTemporalMarker(0, () -> robot.shooter.setShooter(0))
                .addTemporalMarker(0, () -> robot.arm.setArm(BACK))
                .lineToLinearHeading(NONE_DELAY)
                .build();
        Trajectory nonePark = robot.drive.trajectoryBuilder(noneDelay.end())
                .lineToLinearHeading(NONE_PARK)
                .build();

        Trajectory singleDropFirstWobble = robot.drive.trajectoryBuilder(START_POSE)
                .addTemporalMarker(1, () -> robot.shooter.setShooter(R_SHOOTER_GOAL_POWER))
                .back(26)
                .splineToConstantHeading(new Vector2d(SINGLE_DROP_FIRST_WOBBLE.getX(), SINGLE_DROP_FIRST_WOBBLE.getY()), SINGLE_DROP_FIRST_WOBBLE.getHeading())
                .build();
        Trajectory singleGoal = robot.drive.trajectoryBuilder(singleDropFirstWobble.end())
                .addTemporalMarker(0, () -> robot.arm.setArm(DOWN))
                .addTemporalMarker(1, () -> robot.arm.setClaw(CLOSED))
                .lineToLinearHeading(SINGLE_GOAL)
                .build();
        Trajectory singlePickUpRing = robot.drive.trajectoryBuilder(singleGoal.end())
                .addTemporalMarker(0, () -> robot.intake.setIntake(R_INTAKE_SPEED))
                .addTemporalMarker(0, () -> robot.shooter.setShooter(R_SHOOTER_GOAL_POWER))
                .addTemporalMarker(0, () -> robot.arm.setArm(BACK))
                .lineToLinearHeading(SINGLE_PICK_UP_RING)
                .build();
        Trajectory singleGoal2 = robot.drive.trajectoryBuilder(singlePickUpRing.end())
                .lineToLinearHeading(SINGLE_GOAL_2)
                .build();
        Trajectory singleDelay = robot.drive.trajectoryBuilder(singleGoal2.end())
                .addTemporalMarker(0, () -> robot.intake.setIntake(0))
                .addTemporalMarker(0, () -> robot.shooter.setShooter(0))
                .lineToLinearHeading(SINGLE_DELAY)
                .build();
        Trajectory singlePark = robot.drive.trajectoryBuilder(singleDelay.end())
                .lineToLinearHeading(SINGLE_PARK)
                .build();

        Trajectory quadDropFirstWobble = robot.drive.trajectoryBuilder(START_POSE)
                .addTemporalMarker(1.5, () -> robot.shooter.setShooter(R_SHOOTER_GOAL_POWER))
                .lineToLinearHeading(QUAD_DROP_FIRST_WOBBLE)
                .build();
        Trajectory quadGoal = robot.drive.trajectoryBuilder(quadDropFirstWobble.end())
                .addTemporalMarker(0.5, () -> robot.intake.setShield(DOWN))
                .addTemporalMarker(0.5, () -> robot.arm.setArm(DOWN))
                .addTemporalMarker(1.5, () -> robot.arm.setClaw(CLOSED))
                .lineToLinearHeading(QUAD_GOAL)
                .build();
        Trajectory quadPickUpRing = robot.drive.trajectoryBuilder(quadGoal.end())
                .addTemporalMarker(0, () -> robot.intake.setIntake(R_INTAKE_SPEED))
                .addTemporalMarker(0, () -> robot.arm.setArm(BACK))
                .lineToLinearHeading(QUAD_PICK_UP_RING, getVelocityConstraint(15), getAccelerationConstraint())
                .build();
        Trajectory quadGoal2 = robot.drive.trajectoryBuilder(quadPickUpRing.end())
                .lineToLinearHeading(QUAD_GOAL_2)
                .build();
        Trajectory quadRings = robot.drive.trajectoryBuilder(quadGoal2.end())
                .lineToLinearHeading(QUAD_RINGS, getVelocityConstraint(15), getAccelerationConstraint())
                .addTemporalMarker(1.5, () -> robot.intake.setShield(UP))
                .build();
        Trajectory quadGoal3 = robot.drive.trajectoryBuilder(quadRings.end())
                .lineToLinearHeading(QUAD_GOAL_3)
                .build();
        Trajectory quadPark = robot.drive.trajectoryBuilder(quadGoal3.end())
                .addTemporalMarker(0, () -> robot.intake.setIntake(0))
                .addTemporalMarker(0, () -> robot.shooter.setShooter(0))
                .lineToLinearHeading(QUAD_PARK)
                .build();

        switch(stack) {
            case NONE:
                followTrajectory(noneDropFirstWobble);
                setIntake(1, 0.5);
                setIntake(0, 0);
                followTrajectory(noneGoal);
                shootRings(5, false, 3);
                followTrajectory(noneDelay);
                delay(12);
                followTrajectory(nonePark);
                delay(2);
                break;
            case SINGLE:
                followTrajectory(singleDropFirstWobble);
                setIntake(1, 0.5);
                setIntake(0, 0);
                followTrajectory(singleGoal);
                shootRings(3, false, 3);
                followTrajectory(singlePickUpRing);
                followTrajectory(singleGoal2);
                shootRings(3, false, 1);
                followTrajectory(singleDelay);
                delay(6);
                followTrajectory(singlePark);
                delay(2);
                break;
            case QUAD:
                followTrajectory(quadDropFirstWobble);
                setIntake(0.75, 0.5);
                setIntake(0, 0);
                followTrajectory(quadGoal);
                shootRings(2.5, false, 4);
                followTrajectory(quadPickUpRing);
                followTrajectory(quadGoal2);
                shootRings(2.5, false, 2);
                followTrajectory(quadRings);
                followTrajectory(quadGoal3);
                shootRings(2.5, false, 4);
                followTrajectory(quadPark);
                delay(2);
        }
        stopTargetingCamera();
    }
}