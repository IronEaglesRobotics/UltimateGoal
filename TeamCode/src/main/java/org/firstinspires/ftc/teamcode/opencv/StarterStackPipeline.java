package org.firstinspires.ftc.teamcode.opencv;

import org.firstinspires.ftc.teamcode.CVHelpers;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.Constants.ANCHOR;
import static org.firstinspires.ftc.teamcode.Constants.BLUR_SIZE;
import static org.firstinspires.ftc.teamcode.Constants.ERODE_DILATE_ITERATIONS;
import static org.firstinspires.ftc.teamcode.Constants.INVALID_DETECTION;
import static org.firstinspires.ftc.teamcode.Constants.MIN_STARTERSTACK_AREA;
import static org.firstinspires.ftc.teamcode.Constants.ORANGE_LOWER;
import static org.firstinspires.ftc.teamcode.Constants.ORANGE_UPPER;
import static org.firstinspires.ftc.teamcode.Constants.STARTERSTACK_LOCATION;
import static org.firstinspires.ftc.teamcode.Constants.STRUCTURING_ELEMENT;
import static org.firstinspires.ftc.teamcode.Constants.WHITE;

// Class for the pipeline that is used to detect the StarterStack
public class StarterStackPipeline extends OpenCvPipeline  {
    Mat blurred = new Mat();
    Mat hsv = new Mat();
    Mat orangeMask = new Mat();

    private Detection starterStack;

    // Init
    @Override
    public void init(Mat input) {
        starterStack = new Detection(input.size(), MIN_STARTERSTACK_AREA);
    }

    // Process each frame that is received from the webcam
    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.GaussianBlur(input, blurred, BLUR_SIZE, 0);
        Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);

        updateStarterStack(input);

        return input;
    }

    // Update the StarterStack Detection
    private void updateStarterStack(Mat input) {
        // take pixels that are in the color range and put them into a mask, eroding and dilating them to remove white noise
        Core.inRange(hsv, ORANGE_LOWER, ORANGE_UPPER, orangeMask);
        Imgproc.erode(orangeMask, orangeMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);
        Imgproc.dilate(orangeMask, orangeMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);

        // make a new mask that will be used to find the contours of the StarterStack
        Mat starterStackMask = new Mat(hsv.size(), CvType.CV_8U);
        Imgproc.rectangle(starterStackMask, STARTERSTACK_LOCATION, WHITE, -1);
        Core.bitwise_and(orangeMask, starterStackMask, starterStackMask);

        // set the largest detection that was found to be the StarterStack
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(starterStackMask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        starterStack.setContour(CVHelpers.getLargestContour(contours));

        // draw the StarterStack detection as well as where was looked for it on the screen
        Imgproc.rectangle(input, STARTERSTACK_LOCATION, WHITE, 2);
        starterStack.draw(input, WHITE);
    }

    // Get the StarterStack
    public Detection getStarterStack() {
        return starterStack;
    }
}
