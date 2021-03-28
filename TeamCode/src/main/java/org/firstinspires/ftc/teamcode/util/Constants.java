package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.vision.Detection;
import org.firstinspires.ftc.teamcode.vision.PowershotDetection;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
public class Constants {
    // CV Color Constants
    public static Scalar RED = new Scalar(255, 0, 0);
    public static Scalar GREEN = new Scalar(0, 255, 0);
    public static Scalar BLUE = new Scalar(0, 0, 255);
    public static Scalar WHITE = new Scalar(255, 255, 255);

    // CV Color Threshold Constants
    public static Scalar RED_LOWER_1 = new Scalar(0, 85, 80);
    public static Scalar RED_UPPER_1 = new Scalar(15, 255, 255);
    public static Scalar RED_LOWER_2 = new Scalar(165, 85, 80);
    public static Scalar RED_UPPER_2 = new Scalar(180, 255, 255);
    public static Scalar BLUE_LOWER = new Scalar(75, 85, 100);
    public static Scalar BLUE_UPPER = new Scalar(120, 255, 255);
    public static Scalar ORANGE_LOWER = new Scalar(10, 70, 100);
    public static Scalar ORANGE_UPPER = new Scalar(50, 255, 255);

    // CV Structuring Constants
    public static final Mat STRUCTURING_ELEMENT = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
    public static final Point ANCHOR = new Point((STRUCTURING_ELEMENT.cols() / 2f), STRUCTURING_ELEMENT.rows() / 2f);
    public static final int ERODE_DILATE_ITERATIONS = 2;
    public static final Size BLUR_SIZE = new Size(7, 7);

    // CV Camera Constants
    public static final int WEBCAM_WIDTH = 320;
    public static final int WEBCAM_HEIGHT = 240;
    public static final OpenCvCameraRotation WEBCAM_ROTATION = OpenCvCameraRotation.UPRIGHT;

    // CV Detection Constants
    public static double MIN_STARTERSTACK_AREA = 0;
    public static double MIN_STARTERSTACK_SINGLE_AREA = 0.08;
    public static double MIN_STARTERSTACK_QUAD_AREA = 1.10;
    public static double MIN_GOAL_AREA = 0.01;
    public static double MIN_POWERSHOT_AREA = 0.0005;
    public static Rect STARTERSTACK_LOCATION = new Rect(75, 75, 140, 90);
    public static Point POWERSHOT_OFFSET = new Point(-3, -20); // offset from the bottom left of the goal to the top right of the powershot box
    public static Size POWERSHOT_DIMENSIONS = new Size(100, 50);

    // CV Invalid Detection Constants
    public static final Point INVALID_POINT = new Point(Double.MIN_VALUE, Double.MIN_VALUE);
    public static final double INVALID_AREA = -1;
    public static final Detection INVALID_DETECTION = new Detection(new Size(0, 0), 0);
    public static final PowershotDetection INVALID_POWERSHOT_DETECTION = new PowershotDetection(new Size(0, 0), 0);

    // Robot Constants
    public static double WHEEL_SLOW_SPEED = 0.3;
    public static double WHEEL_SPEED = 0.7;
    public static double WHEEL_TURBO_SPEED = 1.0;
    public static double ARM_SPEED = 0.2;
    public static int ARM_DEFAULT_POS = 0;
    public static int ARM_UP_POS = 221;
    public static int ARM_ALMOST_DOWN_POS = 650;
    public static int ARM_DOWN_POS = 750;
    public static double CLAW_CLOSED = 0.13;
    public static double CLAW_OPEN = 0.7;
    public static double INTAKE_SPEED = 0.75;
    public static double INTAKE_SECONDARY_RELATIVE_SPEED = 0.25;
    public static double INTAKE_SHIELD_UP = 0.41;
    public static double INTAKE_SHIELD_DOWN = 0.94;
    public static double SHIELD_SPEED = 0.04;
    public static double SHOOTER_GOAL_POWER = 0.62;
    public static double SHOOTER_POWERSHOT_POWER = 0.57;
    public static double SHOOTER_AUTO_AIM_OFFSET_X = 7;
    public static double PUSHER_CLOSED = 0.35;
    public static double PUSHER_OPEN = 0.55;
    public static double PUSHER_DELAY = 0.15;
    public static double AUTO_AIM_P = 0.8;
    public static double AUTO_AIM_MIN = 0.15;

    // Hardware Name Constants
    public static final String WHEEL_FRONT_LEFT = "frontLeft";
    public static final String WHEEL_FRONT_RIGHT = "frontRight";
    public static final String WHEEL_BACK_LEFT = "backLeft";
    public static final String WHEEL_BACK_RIGHT = "backRight";
    public static final String ARM = "wobbler";
    public static final String CLAW = "claw";
    public static final String INTAKE = "intake";
    public static final String INTAKE_SECONDARY = "secondary";
    public static final String INTAKE_SHIELD = "shield";
    public static final String SHOOTER = "wheel";
    public static final String PUSHER = "pusher";
    public static final String STACK_WEBCAM = "Stack Webcam";
    public static final String TARGETING_WEBCAM = "Targeting Webcam";
    public static final String IMU_SENSOR = "imu";
}
