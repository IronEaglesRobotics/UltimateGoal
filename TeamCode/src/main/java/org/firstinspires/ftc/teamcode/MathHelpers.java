package org.firstinspires.ftc.teamcode;

import android.graphics.Point;

// Math Helper Functions
public class MathHelpers {

    // Get the distance between two points
    public static double getDistanceBetweenTwoPoints (Point start, Point end) {
        float o = end.y - start.y;
        float a = end.x - start.x;
        return Math.sqrt(Math.pow(o, 2) + Math.pow(a, 2));
    }

    // Get the angle between two points
    public static double getAngleBetweenTwoPoints(Point start, Point end) {
        float o = end.y - start.y;
        float a = end.x - start.x;

        double inRads = Math.atan2(o, a);
        return  (inRads >= 0 ? inRads : inRads + (2 * Math.PI)) * 180 / Math.PI;
    }

    // Changes the gyro result from [-179,180] to [0,359]
    public static float piTo2Pi(float angle) {
        return (angle + 360) % 360;
    }

    // Check if an angle is the range of the whole circle
    public static boolean isInRange2pi(float angle, float target, float window) {
        float min = piTo2Pi(target - window);
        float max = piTo2Pi(target + window);
        angle = piTo2Pi(angle);

        return angle > min && angle < max;
    }
}