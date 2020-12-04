package org.firstinspires.ftc.teamcode;

import android.graphics.Point;

//Self explanatory. Put all math helper functions here.
public class MathHelpers {

    public static double getDistanceBetweenTwoPoints (Point start, Point end) {
        float o = end.y - start.y;
        float a = end.x - start.x;
        return Math.sqrt(Math.pow(o, 2) + Math.pow(a, 2));
    }

    public static double getAngleBetweenTwoPoints(Point start, Point end) {
        float o = end.y - start.y;
        float a = end.x - start.x;

        double inRads = Math.atan2(o, a);
        return  (inRads >= 0 ? inRads : inRads + (2 * Math.PI)) * 180 / Math.PI;
    }

    public static float piTo2Pi(float angle) {
        return (angle + 360) % 360;
    }

    public static boolean isInRange2pi(float angle, float target, float window) {
        float min = piTo2Pi(target - window);
        float max = piTo2Pi(target + window);
        angle = piTo2Pi(angle);

        return angle > min && angle < max;
    }
}