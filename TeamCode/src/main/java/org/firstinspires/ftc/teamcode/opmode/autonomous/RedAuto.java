package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.util.Auto;
import org.firstinspires.ftc.teamcode.util.enums.Alliance;
import org.firstinspires.ftc.teamcode.util.enums.StarterStack;

import static org.firstinspires.ftc.teamcode.util.Configurables.INTAKE_SPEED;
import static org.firstinspires.ftc.teamcode.util.Configurables.SHOOTER_GOAL_POWER;
import static org.firstinspires.ftc.teamcode.util.enums.Position.ALMOST_DOWN;
import static org.firstinspires.ftc.teamcode.util.enums.Position.BACK;
import static org.firstinspires.ftc.teamcode.util.enums.Position.CLOSED;
import static org.firstinspires.ftc.teamcode.util.enums.Position.DOWN;
import static org.firstinspires.ftc.teamcode.util.enums.Position.OPEN;
import static org.firstinspires.ftc.teamcode.util.enums.Position.UP;

@Config
@Autonomous(name = "Red Autonomous", group = "Competition", preselectTeleOp = "Red TeleOp")
public class RedAuto extends Auto {
    public static Pose2d START_POSE = new Pose2d(-63.5, -55.75, Math.toRadians(180));

    public static Pose2d NONE_DROP_FIRST_WOBBLE     = new Pose2d(0, -55.75, Math.toRadians(135));
    public static Pose2d NONE_POWERSHOTS            = new Pose2d(-8, -44, Math.toRadians(0));
    public static Pose2d NONE_PICK_UP_SECOND_WOBBLE = new Pose2d(-38, -34.5, Math.toRadians(0));
    public static Pose2d NONE_DROP_SECOND_WOBBLE    = new Pose2d(-2, -43, Math.toRadians(135));
    public static Pose2d NONE_PARK                  = new Pose2d(6, -24, Math.toRadians(180));

    public static Pose2d SINGLE_DROP_FIRST_WOBBLE     = new Pose2d(19, -41, Math.toRadians(0));
    public static Pose2d SINGLE_POWERSHOTS            = new Pose2d(-8, -44, Math.toRadians(0));
    public static Pose2d SINGLE_RING                  = new Pose2d(-8, -39, Math.toRadians(0));
    public static Pose2d SINGLE_PICK_UP_SECOND_WOBBLE = new Pose2d(-40, -34.5, Math.toRadians(0));
    public static Pose2d SINGLE_GOAL                  = new Pose2d(-8, -44, Math.toRadians(0));
    public static Pose2d SINGLE_DROP_SECOND_WOBBLE    = new Pose2d(11.5, -26, Math.toRadians(180));
    public static Pose2d SINGLE_PARK                  = new Pose2d(6, -24, Math.toRadians(180));

    public static Pose2d QUAD_DROP_FIRST_WOBBLE       = new Pose2d(44, -55.75, Math.toRadians(135));
    public static Pose2d QUAD_GOAL                    = new Pose2d(-8, -44, Math.toRadians(0));
    public static Pose2d QUAD_RING                    = new Pose2d(-8, -38, Math.toRadians(0));
    public static Pose2d QUAD_PICK_UP_RING            = new Pose2d(-25, -38, Math.toRadians(0));
    public static Pose2d QUAD_GOAL_2                  = new Pose2d(-8, -44, Math.toRadians(0));
    public static Pose2d QUAD_PICK_UP_SECOND_WOBBLE   = new Pose2d(-42, -35, Math.toRadians(0));
    public static Pose2d QUAD_GOAL_3                  = new Pose2d(-8, -44, Math.toRadians(0));
    public static Pose2d QUAD_DROP_SECOND_WOBBLE      = new Pose2d(32, -53, Math.toRadians(180));
    public static Pose2d QUAD_PARK                    = new Pose2d(6, -24, Math.toRadians(180));

    @Override
    public void setAlliance() {
        this.alliance = Alliance.RED;
    }

    @Override
    public void buildSteps(StarterStack stack) {
        robot.drive.setPoseEstimate(START_POSE);

        Trajectory noneDropFirstWobble = robot.drive.trajectoryBuilder(START_POSE)
                .addTemporalMarker(0.5, () -> robot.shooter.setShooter(SHOOTER_GOAL_POWER))
                .lineToLinearHeading(NONE_DROP_FIRST_WOBBLE)
                .build();
        Trajectory nonePowershots = robot.drive.trajectoryBuilder(noneDropFirstWobble.end())
                .addTemporalMarker(0, () -> robot.arm.setArm(DOWN))
                .addTemporalMarker(1, () -> robot.arm.setClaw(OPEN))
                .lineToLinearHeading(NONE_POWERSHOTS)
                .build();
        Trajectory nonePickUpSecondWobble = robot.drive.trajectoryBuilder(nonePowershots.end())
                .addTemporalMarker(0, () -> robot.shooter.setShooter(0))
                .addTemporalMarker(1.1, () -> robot.arm.setClaw(CLOSED))
                .lineToLinearHeading(NONE_PICK_UP_SECOND_WOBBLE)
                .build();
        Trajectory noneDropSecondWobble = robot.drive.trajectoryBuilder(nonePickUpSecondWobble.end())
                .addTemporalMarker(2.5, () -> robot.arm.setClaw(OPEN))
                .lineToLinearHeading(NONE_DROP_SECOND_WOBBLE)
                .build();
        Trajectory nonePark = robot.drive.trajectoryBuilder(noneDropSecondWobble.end())
                .addTemporalMarker(0, () -> robot.arm.setArm(BACK))
                .addTemporalMarker(0.5, () -> robot.arm.setClaw(CLOSED))
                .lineToLinearHeading(NONE_PARK)
                .build();

        Trajectory singleDropFirstWobble = robot.drive.trajectoryBuilder(START_POSE)
                .addTemporalMarker(1, () -> robot.shooter.setShooter(SHOOTER_GOAL_POWER))
                .back(26)
                .splineToConstantHeading(new Vector2d(SINGLE_DROP_FIRST_WOBBLE.getX(), SINGLE_DROP_FIRST_WOBBLE.getY()), SINGLE_DROP_FIRST_WOBBLE.getHeading())
                .build();
        Trajectory singlePowershots = robot.drive.trajectoryBuilder(singleDropFirstWobble.end())
                .addTemporalMarker(0, () -> robot.arm.setArm(DOWN))
                .addTemporalMarker(1, () -> robot.arm.setClaw(OPEN))
                .lineToLinearHeading(SINGLE_POWERSHOTS)
                .build();
        Trajectory singleRing = robot.drive.trajectoryBuilder(singlePowershots.end())
                .lineToLinearHeading(SINGLE_RING)
                .build();
        Trajectory singlePickUpSecondWobble = robot.drive.trajectoryBuilder(singleRing.end())
                .addTemporalMarker(0, () -> robot.intake.setIntake(INTAKE_SPEED))
                .addTemporalMarker(0, () -> robot.shooter.setShooter(SHOOTER_GOAL_POWER))
                .addTemporalMarker(1, () -> robot.arm.setClaw(CLOSED))
                .lineToLinearHeading(SINGLE_PICK_UP_SECOND_WOBBLE)
                .build();
        Trajectory singleGoal = robot.drive.trajectoryBuilder(singlePickUpSecondWobble.end())
                .addTemporalMarker(0, () -> robot.arm.setArm(ALMOST_DOWN))
                .lineToLinearHeading(SINGLE_GOAL)
                .build();
        Trajectory singleDropSecondWobble = robot.drive.trajectoryBuilder(singleGoal.end())
                .addTemporalMarker(0, () -> robot.intake.setIntake(0))
                .addTemporalMarker(0, () -> robot.shooter.setShooter(0))
                .addTemporalMarker(3.5, () -> robot.arm.setClaw(OPEN))
                .lineToLinearHeading(SINGLE_DROP_SECOND_WOBBLE)
                .build();
        Trajectory singlePark = robot.drive.trajectoryBuilder(singleDropSecondWobble.end())
                .addTemporalMarker(0, () -> robot.arm.setArm(BACK))
                .addTemporalMarker(0.5, () -> robot.arm.setClaw(CLOSED))
                .lineToLinearHeading(SINGLE_PARK)
                .build();

        Trajectory quadDropFirstWobble = robot.drive.trajectoryBuilder(START_POSE)
                .addTemporalMarker(1.5, () -> robot.shooter.setShooter(SHOOTER_GOAL_POWER))
                .lineToLinearHeading(QUAD_DROP_FIRST_WOBBLE)
                .build();
        Trajectory quadGoal = robot.drive.trajectoryBuilder(quadDropFirstWobble.end())
                .addTemporalMarker(0.5, () -> robot.intake.setShield(DOWN))
                .addTemporalMarker(0.5, () -> robot.arm.setArm(DOWN))
                .addTemporalMarker(1.5, () -> robot.arm.setClaw(OPEN))
                .lineToLinearHeading(QUAD_GOAL)
                .build();
        Trajectory quadRing = robot.drive.trajectoryBuilder(quadGoal.end())
                .lineToLinearHeading(QUAD_RING)
                .build();
        Trajectory quadPickUpRing = robot.drive.trajectoryBuilder(quadRing.end())
                .addTemporalMarker(0, () -> robot.intake.setIntake(INTAKE_SPEED))
                .lineToLinearHeading(QUAD_PICK_UP_RING, getVelocityConstraint(15), getAccelerationConstraint())
                .build();
        Trajectory quadGoal2 = robot.drive.trajectoryBuilder(quadPickUpRing.end())
                .lineToLinearHeading(QUAD_GOAL_2)
                .build();
        Trajectory quadPickUpSecondWobble = robot.drive.trajectoryBuilder(quadGoal2.end())
                .addTemporalMarker(2.75, () -> robot.arm.setClaw(CLOSED))
                .lineToLinearHeading(QUAD_PICK_UP_SECOND_WOBBLE, getVelocityConstraint(15), getAccelerationConstraint())
                .build();
        Trajectory quadGoal3 = robot.drive.trajectoryBuilder(quadPickUpSecondWobble.end())
                .addTemporalMarker(0, () -> robot.arm.setArm(ALMOST_DOWN))
                .lineToLinearHeading(QUAD_GOAL_3)
                .build();
        Trajectory quadDropSecondWobble = robot.drive.trajectoryBuilder(quadGoal3.end())
                .addTemporalMarker(0, () -> robot.intake.setIntake(0))
                .addTemporalMarker(0, () -> robot.intake.setShield(UP))
                .addTemporalMarker(0, () -> robot.shooter.setShooter(0))
                .addTemporalMarker(3, () -> robot.arm.setClaw(OPEN))
                .lineToLinearHeading(QUAD_DROP_SECOND_WOBBLE)
                .build();
        Trajectory quadPark = robot.drive.trajectoryBuilder(quadDropSecondWobble.end())
                .addTemporalMarker(0, () -> robot.arm.setArm(BACK))
                .addTemporalMarker(0.5, () -> robot.arm.setClaw(CLOSED))
                .lineToLinearHeading(QUAD_PARK)
                .build();

        switch(stack) {
            case NONE:
                followTrajectory(noneDropFirstWobble);
                setIntake(1, 0.5);
                setIntake(0, 0);
                followTrajectory(nonePowershots);
                shootRings(10, false, 3);
                followTrajectory(nonePickUpSecondWobble);
                followTrajectory(noneDropSecondWobble);
                followTrajectory(nonePark);
                delay(2);
                break;
            case SINGLE:
                followTrajectory(singleDropFirstWobble);
                setIntake(1, 0.5);
                setIntake(0, 0);
                followTrajectory(singlePowershots);
                shootRings(6, false, 3);
                followTrajectory(singleRing);
                followTrajectory(singlePickUpSecondWobble);
                followTrajectory(singleGoal);
                shootRings(3, false, 1);
                followTrajectory(singleDropSecondWobble);
                followTrajectory(singlePark);
                delay(2);
                break;
            case QUAD:
                followTrajectory(quadDropFirstWobble);
                setIntake(1, 0.5);
                setIntake(0, 0);
                followTrajectory(quadGoal);
                shootRings(6, false, 4);
                followTrajectory(quadRing);
                followTrajectory(quadPickUpRing);
                followTrajectory(quadGoal2);
                shootRings(3, false, 2);
                followTrajectory(quadPickUpSecondWobble);
                followTrajectory(quadGoal3);
                shootRings(6, false, 4);
                followTrajectory(quadDropSecondWobble);
                followTrajectory(quadPark);
                delay(2);
        }
        stopTargetingCamera();
    }
}