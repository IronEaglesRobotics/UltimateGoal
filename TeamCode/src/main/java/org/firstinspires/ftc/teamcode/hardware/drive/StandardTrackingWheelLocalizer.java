package org.firstinspires.ftc.teamcode.hardware.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.util.Constants.WHEEL_BACK_RIGHT;
import static org.firstinspires.ftc.teamcode.util.Constants.WHEEL_FRONT_LEFT;
import static org.firstinspires.ftc.teamcode.util.Constants.WHEEL_FRONT_RIGHT;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
//@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.6889764; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 15.768; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 7; // in; offset of the lateral wheel //6.762

    public static double X_MULTIPLIER = 1.013; // Multiplier in the X direction //1.02857 //88.77,88.97,88.84 --> 88.86
    public static double Y_MULTIPLIER = 1.013; // Multiplier in the Y direction
    private Encoder leftEncoder, rightEncoder, frontEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(-0.096, 7.884, 0), // left //8.179
                new Pose2d(0.096, -7.884, 0), // right
                new Pose2d(FORWARD_OFFSET, -0.253, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, WHEEL_FRONT_LEFT));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, WHEEL_BACK_RIGHT));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, WHEEL_FRONT_RIGHT));

        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        leftEncoder.setDirection(Encoder.Direction.REVERSE);

    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }
}
