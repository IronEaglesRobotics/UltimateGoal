package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.util.Auto;
import org.firstinspires.ftc.teamcode.util.enums.Alliance;
import org.firstinspires.ftc.teamcode.util.enums.Position;
import org.firstinspires.ftc.teamcode.util.enums.StarterStack;

import static org.firstinspires.ftc.teamcode.util.Configurables.R_SHOOTER_GOAL_POWER;

@Autonomous(name = "Blue Inside 1 Wobble", group = "Competition", preselectTeleOp = "Blue TeleOp")
public class BlueInside1W extends Auto {
    public static Pose2d START_POSE = new Pose2d(-62.5, 18.9, Math.toRadians(180));

    public static Pose2d STRAFE = new Pose2d(-62.5, 11, Math.toRadians(180));
    public static Pose2d SHOOT = new Pose2d(-3, 11, Math.toRadians(30));
    public static Pose2d DELAY = new Pose2d(52, 17, Math.toRadians(-90));

    public static Pose2d NONE_DROP_WOBBLE = new Pose2d(26, 58, Math.toRadians(0));
    public static Pose2d NONE_GOING_BACK = new Pose2d(42, 37, Math.toRadians(-90));
    public static Pose2d NONE_PARK = new Pose2d(10, 11, Math.toRadians(180));

    public static Pose2d SINGLE_DROP_WOBBLE = new Pose2d(32, 24, Math.toRadians(-90));
    public static Pose2d SINGLE_PARK = new Pose2d(8, 12, Math.toRadians(180));

    public static Pose2d QUAD_DROP_WOBBLE = new Pose2d(55, 50, Math.toRadians(-90));
    public static Pose2d QUAD_PARK = new Pose2d(8, 12, Math.toRadians(180));

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

        Trajectory strafe = robot.drive.trajectoryBuilder(START_POSE)
                .addTemporalMarker(0, () -> robot.shooter.setShooter(R_SHOOTER_GOAL_POWER))
                .addTemporalMarker(0, () -> robot.intake.setIntake(-0.01))
                .lineToLinearHeading(STRAFE)
                .build();
        Trajectory shoot = robot.drive.trajectoryBuilder(strafe.end())
                .addTemporalMarker(0, () -> robot.arm.setArm(Position.ALMOST_DOWN))
                .addTemporalMarker(1.5, () -> robot.arm.setClaw(Position.CLOSED))
                .addTemporalMarker(2.5, () -> robot.arm.setArm(Position.BACK))
                .lineToLinearHeading(SHOOT)
                .build();
        Trajectory delay = robot.drive.trajectoryBuilder(shoot.end())
                .addTemporalMarker(0, () -> robot.shooter.setShooter(0))
                .lineToLinearHeading(DELAY)
                .build();

        Trajectory noneWobble = robot.drive.trajectoryBuilder(delay.end(), true)
                .back(10)
                .splineToSplineHeading(NONE_DROP_WOBBLE, Math.toRadians(-90))
                .build();
        Trajectory nonePark = robot.drive.trajectoryBuilder(noneWobble.end())
                .splineTo(new Vector2d(NONE_GOING_BACK.getX(), NONE_GOING_BACK.getY()), NONE_GOING_BACK.getHeading())
                .splineTo(new Vector2d(NONE_PARK.getX(), NONE_PARK.getY()), NONE_PARK.getHeading())
                .build();

        Trajectory singleWobble = robot.drive.trajectoryBuilder(delay.end())
                .lineToLinearHeading(SINGLE_DROP_WOBBLE)
                .build();
        Trajectory singlePark = robot.drive.trajectoryBuilder(singleWobble.end())
                .splineTo(new Vector2d(SINGLE_PARK.getX(), SINGLE_PARK.getY()), SINGLE_PARK.getHeading())
                .build();

        Trajectory quadWobble = robot.drive.trajectoryBuilder(delay.end())
                .lineToLinearHeading(QUAD_DROP_WOBBLE)
                .build();
        Trajectory quadPark = robot.drive.trajectoryBuilder(quadWobble.end())
                .splineTo(new Vector2d(QUAD_PARK.getX(), QUAD_PARK.getY()), QUAD_PARK.getHeading())
                .build();

        followTrajectory(strafe);
        followTrajectory(shoot);
        shootRings(7, false, 3);
//        shootRings(4, false,1);
//        setPusher(0.4, Position.OPEN);
//        setPusher(0.4, Position.CLOSED);
//        setPusher(0.4, Position.OPEN);
//        setPusher(0.4, Position.CLOSED);
//        setPusher(0.4, Position.OPEN);
//        setPusher(0.4, Position.CLOSED);
//        setPusher(0, Position.OPEN);
        followTrajectory(delay);
        switch(stack) {
            case NONE:
                delay(5);
                followTrajectory(noneWobble);
                setIntake(1, 0.5);
                setIntake(0, 0);
                followTrajectory(nonePark);
                break;
            case SINGLE:
                delay(7);
                followTrajectory(singleWobble);
                setIntake(1, 0.5);
                setIntake(0, 0);
                followTrajectory(singlePark);
                break;
            case QUAD:
//                delay(7);
                followTrajectory(quadWobble);
                setIntake(1, 0.5);
                setIntake(0, 0);
                followTrajectory(quadPark);
        }
        stopTargetingCamera();
    }
}