package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.opencv.Detection;
import org.firstinspires.ftc.teamcode.opencv.PowershotDetection;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraRotation;

// Class used to keep track of each constant value
public class Constants {
    // CV Color Constants
    public static final Scalar RED = new Scalar(255, 0, 0);
    public static final Scalar GREEN = new Scalar(0, 255, 0);
    public static final Scalar BLUE = new Scalar(0, 0, 255);
    public static final Scalar WHITE = new Scalar(255, 255, 255);

    // CV Color Threshold Constants
    public static final Scalar RED_LOWER_1 = new Scalar(0, 90, 90);
    public static final Scalar RED_UPPER_1 = new Scalar(15, 255, 255);
    public static final Scalar RED_LOWER_2 = new Scalar(165, 90, 90);
    public static final Scalar RED_UPPER_2 = new Scalar(180, 255, 255);
    public static final Scalar BLUE_LOWER = new Scalar(75, 85, 100);
    public static final Scalar BLUE_UPPER = new Scalar(120, 255, 255);
    public static final Scalar ORANGE_LOWER = new Scalar(10, 85, 100);
    public static final Scalar ORANGE_UPPER = new Scalar(25, 255, 255);

    // CV Structuring Constants
    public static final Mat STRUCTURING_ELEMENT = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
    public static final Point ANCHOR = new Point((STRUCTURING_ELEMENT.cols() / 2f), STRUCTURING_ELEMENT.rows() / 2f);
    public static final int ERODE_DILATE_ITERATIONS = 2;
    public static final Size BLUR_SIZE = new Size(11, 11);

    // CV Camera Constants
    public static final int WEBCAM_WIDTH = 320;
    public static final int WEBCAM_HEIGHT = 240;
    public static final OpenCvCameraRotation WEBCAM_ROTATION = OpenCvCameraRotation.UPRIGHT;

    // CV Detection Constants
    public static final double MIN_STARTERSTACK_AREA = 0.005;
    public static final double MIN_STARTERSTACK_SINGLE_AREA = 0.7;
    public static final double MIN_STARTERSTACK_QUAD_AREA = 1.85;
    public static final double MIN_GOAL_AREA = 0.01;
    public static final double MIN_POWERSHOT_AREA = 0.001 ;
    public static final Rect STARTERSTACK_LOCATION = new Rect(0, 75, 160, 90);
    public static final Size GOAL_DIMENSIONS_IN = new Size(24, 15.5);
    public static final Point POWERSHOT_OFFSET_IN = new Point(-29, -11);
    public static final Size POWERSHOT_DIMENSIONS_IN = new Size(30, 25);

    // CV Invalid Detection Constants
    public static final Point INVALID_POINT = new Point(Double.MIN_VALUE, Double.MIN_VALUE);
    public static final double INVALID_AREA = -1;
    public static final Detection INVALID_DETECTION = new Detection(new Size(0, 0), 0);
    public static final PowershotDetection INVALID_POWERSHOT_DETECTION = new PowershotDetection(new Size(0, 0), 0);

    // Robot Constants
    public static final double WHEEL_DIAMETER = 4.0;
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    public static final double WHEEL_SLOW_SPEED = 0.3;
    public static final double WHEEL_SPEED = 0.7;
    public static final double WHEEL_TURBO_SPEED = 1.0;
    public static final double ARM_SPEED = 0.25;
    public static final int ARM_DOWN_POS = -756;
    public static final int ARM_UP_POS = -221;
    public static final double CLAW_MIN = 0.05;
    public static final double CLAW_MAX = 0.8;
    public static final double INTAKE_MAX_SPEED = 0.75;
    public static final double SECONDARY_INTAKE_RELATIVE_SPEED = 0.5;
    public static final double SHOOTER_POWER = 0.62;
    public static final double POWERSHOT_SHOOTER_POWER = 0.57;
    public static final int AUTO_AIM_OFFSET_X = 10;
    public static final double PUSHER_MIN = 0.22;
    public static final double PUSHER_MAX = 0.4;

    // Hardware Name Constants
    public static final String WHEEL_FRONT_LEFT = "frontLeft";
    public static final String WHEEL_FRONT_RIGHT = "frontRight";
    public static final String WHEEL_BACK_LEFT = "backLeft";
    public static final String WHEEL_BACK_RIGHT = "backRight";
    public static final String ARM = "wobbler";
    public static final String CLAW = "claw";
    public static final String INTAKE = "intake";
    public static final String SECONDARY_INTAKE = "secondary";
    public static final String SHOOTER = "shooter";
    public static final String PUSHER = "pusher";
    public static final String STACK_WEBCAM = "Stack Webcam";
    public static final String TARGETING_WEBCAM = "Targeting Webcam";
    public static final String IMU_SENSOR = "imu";

    // Enums for StarterStack, Motor, and Servo Positions
    public enum StarterStack {
        NONE, SINGLE, QUAD
    }
    public enum ServoPosition {
        OPEN, CLOSED
    }
    public enum ArmPosition {
        UP, DOWN
    }
}
