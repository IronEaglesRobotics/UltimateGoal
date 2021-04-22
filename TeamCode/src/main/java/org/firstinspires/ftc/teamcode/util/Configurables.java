package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

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
    public static double INTAKE_SHIELD_UP = 0.05;//0.41
    public static double INTAKE_SHIELD_DOWN = 0.95;//0.93
    public static double INTAKE_SHIELD_SPEED = 0.04;
    public static double SHOOTER_GOAL_POWER = 0.62;
    public static double SHOOTER_MID_GOAL_POWER = 0.54;
    public static double SHOOTER_POWERSHOT_POWER = 0.57;
    public static double SHOOTER_AUTO_AIM_OFFSET_X = 5;
    public static double PUSHER_CLOSED = 0.35;
    public static double PUSHER_OPEN = 0.55;
    public static double PUSHER_DELAY = 0.15;
    public static double AUTO_AIM_WAIT = 0.02;
    public static double AUTO_AIM_MAX_ERROR = 50;
    public static double AUTO_AIM_A = 0.12;
    public static double AUTO_AIM_H = 0;
    public static double AUTO_AIM_EXP = 3.0;

    // CV Color Threshold Constants
    public static Color CAMERA_RED_GOAL_LOWER = new Color(165, 85, 80);
    public static Color CAMERA_RED_GOAL_UPPER = new Color(15, 255, 255);
    public static Color CAMERA_RED_POWERSHOT_LOWER = new Color(165, 85, 80);
    public static Color CAMERA_RED_POWERSHOT_UPPER = new Color(15, 255, 255);
    public static Color CAMERA_BLUE_GOAL_LOWER = new Color(75, 85, 100);
    public static Color CAMERA_BLUE_GOAL_UPPER = new Color(120, 255, 255);
    public static Color CAMERA_BLUE_POWERSHOT_LOWER = new Color(75, 50, 50);
    public static Color CAMERA_BLUE_POWERSHOT_UPPER = new Color(120, 255, 255);
    public static Color CAMERA_ORANGE_LOWER = new Color(10, 70, 100);
    public static Color CAMERA_ORANGE_UPPER = new Color(50, 255, 255);
}
