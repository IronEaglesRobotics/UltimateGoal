package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.vision.Detection;
import org.firstinspires.ftc.teamcode.vision.PowershotDetection;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Constants {
    // CV Color Constants
    public static Scalar RED = new Scalar(255, 0, 0);
    public static Scalar GREEN = new Scalar(0, 255, 0);
    public static Scalar BLUE = new Scalar(0, 0, 255);
    public static Scalar WHITE = new Scalar(255, 255, 255);

    // CV Structuring Constants
    public static final Mat STRUCTURING_ELEMENT = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
    public static final Point ANCHOR = new Point((STRUCTURING_ELEMENT.cols() / 2f), STRUCTURING_ELEMENT.rows() / 2f);
    public static final int ERODE_DILATE_ITERATIONS = 2;
    public static final Size BLUR_SIZE = new Size(7, 7);

    // CV Camera Constants
    public static final int WEBCAM_WIDTH = 320;
    public static final int WEBCAM_HEIGHT = 240;
    public static final OpenCvCameraRotation WEBCAM_ROTATION = OpenCvCameraRotation.UPRIGHT;

//    // CV Detection Constants
//    public static double MIN_STARTERSTACK_AREA = 0;
//    public static double MIN_STARTERSTACK_SINGLE_AREA = 0.08;
//    public static double MIN_STARTERSTACK_QUAD_AREA = 1.10;
//    public static double MIN_GOAL_AREA = 0.01;
//    public static double MIN_POWERSHOT_AREA = 0.0005;
//    public static Rect STARTERSTACK_LOCATION = new Rect(75, 50, 190, 90);
//    public static Point POWERSHOT_OFFSET = new Point(-3, -20); // offset from the bottom left of the goal to the top right of the powershot box
//    public static Size POWERSHOT_DIMENSIONS = new Size(100, 50);

    // CV Invalid Detection Constants
    public static final Point INVALID_POINT = new Point(Double.MIN_VALUE, Double.MIN_VALUE);
    public static final double INVALID_AREA = -1;
    public static final Detection INVALID_DETECTION = new Detection(new Size(0, 0), 0);
    public static final PowershotDetection INVALID_POWERSHOT_DETECTION = new PowershotDetection(new Size(0, 0), 0);

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
