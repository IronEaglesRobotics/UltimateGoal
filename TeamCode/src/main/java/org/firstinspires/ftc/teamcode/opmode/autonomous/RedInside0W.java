package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.util.Auto;
import org.firstinspires.ftc.teamcode.util.enums.Alliance;
import org.firstinspires.ftc.teamcode.util.enums.Position;
import org.firstinspires.ftc.teamcode.util.enums.StarterStack;

import static org.firstinspires.ftc.teamcode.util.Configurables.R_SHOOTER_GOAL_POWER;

@Autonomous(name = "Red Inside 0 Wobble", group = "Competition", preselectTeleOp = "Red TeleOp")
public class RedInside0W extends Auto {
    public static Pose2d START_POSE = new Pose2d(-59.5, -18.0, Math.toRadians(45));

    public static Pose2d SHOOT = new Pose2d(-8, -12, Math.toRadians(-30));
    public static Pose2d PARK  = new Pose2d(10, -12, Math.toRadians(180));

    @Override
    public void setAlliance() {
        this.alliance = Alliance.RED;
    }

    @Override
    public void setCamera() {
        this.checkForStarterStack = false;
    }

    @Override
    public void buildSteps(StarterStack stack) {
        robot.drive.setPoseEstimate(START_POSE);

        Trajectory shoot = robot.drive.trajectoryBuilder(START_POSE)
                .addTemporalMarker(0, () -> robot.shooter.setShooter(R_SHOOTER_GOAL_POWER))
                .addTemporalMarker(0.5, () -> robot.arm.setArm(Position.ALMOST_DOWN))
                .addTemporalMarker(1.5, () -> robot.arm.setClaw(Position.CLOSED))
                .lineToLinearHeading(SHOOT)
                .build();
        Trajectory park = robot.drive.trajectoryBuilder(shoot.end())
                .addTemporalMarker(0, () -> robot.shooter.setShooter(0))
                .addTemporalMarker(0, () -> robot.arm.setArm(Position.BACK))
                .lineToLinearHeading(PARK)
                .build();

        delay(22);
        followTrajectory(shoot);
        shootRings(3.5, false, 3);
        followTrajectory(park);
        stopTargetingCamera();
    }
}