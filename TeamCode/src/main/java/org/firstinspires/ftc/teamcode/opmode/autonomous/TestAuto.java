package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.util.Auto;
import org.firstinspires.ftc.teamcode.util.enums.Alliance;
import org.firstinspires.ftc.teamcode.util.enums.StarterStack;

@Autonomous(name = "Test Auto", group = "Development", preselectTeleOp = "")
public class TestAuto extends Auto {
    public static Pose2d START_POSE = new Pose2d(0, 0, Math.toRadians(0));

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

        Trajectory move1 = robot.drive.trajectoryBuilder(START_POSE)
                .forward(48)
                .build();
        Trajectory move2 = robot.drive.trajectoryBuilder(move1.end())
                .strafeLeft(48)
                .build();

        followTrajectory(move1);
        delay(2);
        followTrajectory(move2);
        stopTargetingCamera();
    }
}