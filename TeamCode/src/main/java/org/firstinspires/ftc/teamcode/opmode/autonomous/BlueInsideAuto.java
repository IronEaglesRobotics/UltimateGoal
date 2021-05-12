package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.util.Auto;
import org.firstinspires.ftc.teamcode.util.enums.Alliance;
import org.firstinspires.ftc.teamcode.util.enums.StarterStack;

import static org.firstinspires.ftc.teamcode.util.Configurables.SHOOTER_GOAL_POWER;

@Config
@Autonomous(name = "Blue Inside Auto", group = "Competition", preselectTeleOp = "Blue TeleOp")
public class BlueInsideAuto extends Auto {
    public static Pose2d START_POSE = new Pose2d(-59.5, 18.0, Math.toRadians(-45));

    public static Pose2d SHOOT = new Pose2d(-8, 12, Math.toRadians(10));
    public static Pose2d PARK  = new Pose2d(10, 6, Math.toRadians(180));//12

    @Override
    public void setAlliance() {
        this.alliance = Alliance.BLUE;
    }

    @Override
    public void setCamera() {
        this.checkForStarterStack = false;
    }

    @Override
    public void buildSteps(StarterStack stack) {
        robot.drive.setPoseEstimate(START_POSE);

        Trajectory shoot = robot.drive.trajectoryBuilder(START_POSE)
                .addTemporalMarker(0, () -> robot.shooter.setShooter(SHOOTER_GOAL_POWER))
                .lineToLinearHeading(SHOOT)
                .build();
        Trajectory park = robot.drive.trajectoryBuilder(shoot.end())
                .addTemporalMarker(0, () -> robot.shooter.setShooter(0))
                .lineToLinearHeading(PARK)
                .build();

        delay(24);
        followTrajectory(shoot);
        shootRings(3.5, false, 3);
        followTrajectory(park);
        stopTargetingCamera();
    }
}