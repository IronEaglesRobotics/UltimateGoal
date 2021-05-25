package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Size;

@Config
public class Configurables {
    // Robot Constants
    public static double ARM_POWER = 0.2;
    public static double ARM_SPEED = 20;
    public static int ARM_DEFAULT_POS = 0;
    public static int ARM_UP_POS = 221;
    public static int ARM_ALMOST_DOWN_POS = 650;
    public static int ARM_DOWN_POS = 750;
    public static double CLAW_CLOSED = 0.13;
    public static double CLAW_OPEN = 0.7;
    public static double INTAKE_SPEED = 0.9;
    public static double INTAKE_SHIELD_UP = 0.17;//0.05
    public static double INTAKE_SHIELD_DOWN = 0.68;//0.95
    public static double INTAKE_SHIELD_SPEED = 0.04;
    public static double SHOOTER_GOAL_POWER = 0.64;
    public static double SHOOTER_MID_GOAL_POWER = 0.54;
    public static double SHOOTER_POWERSHOT_POWER = 0.57;
    public static double PUSHER_CLOSED = 0.35;
    public static double PUSHER_OPEN = 0.55;
    public static double PUSHER_DELAY = 0.15;
    public static double AUTO_AIM_OFFSET_X = 5;
    public static double AUTO_AIM_WAIT = 0.1;
    public static double AUTO_AIM_MAX_ERROR = 50;
    public static double AUTO_AIM_A = 0.05;
    public static double AUTO_AIM_H = 0.4;
    public static double AUTO_AIM_EXP = 2.0;
    public static double AUTO_AIM_XP = 0.014;
    public static double AUTO_AIM_XI = 0.005;
    public static double AUTO_AIM_XD = 0.0015;
//    public static PIDFCoefficients AUTO_AIM_PID = new PIDFCoefficients(1, 0, 0, 0);
    public static double AUTO_AIM_ACCEPTABLE_ERROR = 0.5;

    // CV Color Threshold Constants
    public static Color CAMERA_RED_GOAL_LOWER       = new Color(165, 80, 80);
    public static Color CAMERA_RED_GOAL_UPPER       = new Color(15, 255, 255);
    public static Color CAMERA_RED_POWERSHOT_LOWER  = new Color(165, 80, 80);
    public static Color CAMERA_RED_POWERSHOT_UPPER  = new Color(15, 255, 255);
    public static Color CAMERA_BLUE_GOAL_LOWER      = new Color(75, 40, 80);
    public static Color CAMERA_BLUE_GOAL_UPPER      = new Color(120, 255, 255);
    public static Color CAMERA_BLUE_POWERSHOT_LOWER = new Color(75, 30, 50);
    public static Color CAMERA_BLUE_POWERSHOT_UPPER = new Color(120, 255, 255);
    public static Color CAMERA_ORANGE_LOWER         = new Color(0, 70, 50);
    public static Color CAMERA_ORANGE_UPPER         = new Color(60, 255, 255);
    public static Color CAMERA_BLACK_LOWER          = new Color(0, 0, 0);
    public static Color CAMERA_BLACK_UPPER          = new Color(180, 255, 70);
    public static Color CAMERA_WHITE_LOWER          = new Color(0, 0, 0);
    public static Color CAMERA_WHITE_UPPER          = new Color(180, 70, 255);

    // CV Detection Constants
    public static double CV_MIN_STARTERSTACK_AREA = 0;
    public static double CV_MIN_STARTERSTACK_SINGLE_AREA = 0.08;
    public static double CV_MIN_STARTERSTACK_QUAD_AREA = 1.3;
    public static double CV_MIN_GOAL_AREA = 0.01;
    public static double CV_MIN_POWERSHOT_AREA = 0.001;
    public static Rect CV_STARTERSTACK_LOCATION = new Rect(75, 50, 190, 90);
    public static Point CV_POWERSHOT_OFFSET = new Point(-3, -20); // offset from the bottom left of the goal to the top right of the powershot box (for red)
    public static Size CV_POWERSHOT_DIMENSIONS = new Size(100, 50);
    public static double CV_GOAL_ALLOWABLE_Y_ERROR = 20;
    public static double CV_GOAL_ALLOWABLE_SIZE_ERROR = 1000;
    public static double CV_GOAL_ALLOWABLE_Y_LINE = -100;
}
