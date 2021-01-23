package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.CVHelpers.BLUE;
import static org.firstinspires.ftc.teamcode.CVHelpers.GREEN;
import static org.firstinspires.ftc.teamcode.CVHelpers.RED;
import static org.firstinspires.ftc.teamcode.CVHelpers.getLargestContour;

/*Terms:
* - Mask ... The isolation of a certain range of colors. You're probably better off Googling it.
* - HSV ... Another way of representing color digitally. The standard way is RGB.
* - "Detection" ... The detection class that we built. Is initialized with two variables [see definition].
* - MatOfPoint ... Pay extra attention to this type of MatOf[Tyabpe]. This is what we use to store contours (I think).
* - Erode ... To thin the shapes that make up a mask. OpenCV docs give examples of this.
* - Dilate ... Opposite of erode.
* - "extends OpenCvPipeline" ... This is important. This pipeline class extends FTC's pipeline class. To use EasyOpenCV you need to specify a pipeline, this is ours. You'll see what I mean in CVSandbox.
* - STRUCTURING_ELEMENT ... The shape that controls dilation of an image. Can be a star, ellipse, square, or etc.
*/

class CVPipeline extends OpenCvPipeline
{
    Mat blurred = new Mat();
    Mat hsv = new Mat();
    Mat redMask1 = new Mat();
    Mat redMask2 = new Mat();
    Mat redMask = new Mat();
    Mat blueMask = new Mat();

    final Mat STRUCTURING_ELEMENT = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size((2 * 2) + 1, (2 * 2) + 1));
    final Point ANCHOR = new Point((STRUCTURING_ELEMENT.cols() / 2f), STRUCTURING_ELEMENT.rows() / 2f);
    final double MIN_GOAL_AREA = 0.01;
    final double MIN_POWERSHOT_AREA = 0.001 ;
    final Scalar RED_LOWER_1 = new Scalar(0, 90, 90);
    final Scalar RED_UPPER_1 = new Scalar(15, 255, 255);
    final Scalar RED_LOWER_2 = new Scalar(165, 90, 90);
    final Scalar RED_UPPER_2 = new Scalar(180, 255, 255);
    final Scalar BLUE_LOWER = new Scalar(75, 85, 100);
    final Scalar BLUE_UPPER = new Scalar(120, 255, 255);
    final int ERODE_DILATE_ITERATIONS = 2;
    final Size BLUR_SIZE = new Size(11, 11);

    private Detection red;
    private Detection blue;
    private PowerShotDetection powerShots;

    @Override
    public void init(Mat input) {
        red = new Detection(input.size(), MIN_GOAL_AREA);
        blue = new Detection(input.size(), MIN_GOAL_AREA);
        powerShots = new PowerShotDetection(input.size(), MIN_POWERSHOT_AREA);
    }

    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.GaussianBlur(input, blurred, BLUR_SIZE, 0);
        Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);

        updateRed();
        red.draw(input, RED);

        updateBlue();
        blue.draw(input, BLUE);

        updatePowerShots(input);
        powerShots.draw(input, GREEN);

        return input;
    }

    //Sets `red` to largest red contour.
    private void updateRed() {
        // Get the mask for RED objects. Note that since we're in HSV color space, red appears at
        // both the lower and upper end of the H spectrum. Thus, we have two separate red masks
        // which we then merge into one.
        Core.inRange(hsv, RED_LOWER_1, RED_UPPER_1, redMask1);
        Core.inRange(hsv, RED_LOWER_2, RED_UPPER_2, redMask2);
        Core.add(redMask1, redMask2, redMask);

        //Erode and then dilate the mask to get rid of white noise.
        //This can also be done via. Opening. See:
        //https://docs.opencv.org/master/d9/d61/tutorial_py_morphological_ops.html
        Imgproc.erode(redMask, redMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);
        Imgproc.dilate(redMask, redMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(redMask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        red.setContour(getLargestContour(contours));
    }

    //Sets `blue` to largest blue contour.
    private void updateBlue() {
        // Get the mask for BLUE objects. Note that because blue is isolated in the H spectrum, we
        // do not need two separate sets of lower and upper values.
        Core.inRange(hsv, BLUE_LOWER, BLUE_UPPER, blueMask);

        //Remove noise.
        Imgproc.erode(blueMask, blueMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);
        Imgproc.dilate(blueMask, blueMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(blueMask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        blue.setContour(getLargestContour(contours));
    }

    private void updatePowerShots(Mat input) {
        if (!this.red.isValid()) {
            return;
        }

        Point gc = red.getCenterPx();
        Rect goalBoundingBox = Imgproc.boundingRect(red.getContour());
        double goalUnit = goalBoundingBox.width / PowerShotDetection.GOAL_DIMENSIONS_IN.width;

        Point powerShotCenter = new Point(
                gc.x + goalUnit * PowerShotDetection.POWERSHOT_OFFSET_IN.x,
                gc.y - goalUnit * PowerShotDetection.POWERSHOT_OFFSET_IN.y
        );
        Point powerShotTL = new Point(
                (int)(powerShotCenter.x - (goalUnit * PowerShotDetection.POWERSHOT_DIMENSIONS_IN.width / 2)),
                (int)(powerShotCenter.y - (goalUnit * PowerShotDetection.POWERSHOT_DIMENSIONS_IN.height / 2))
        );
        Point powerShotBR = new Point(
                (int)(powerShotCenter.x + (goalUnit * PowerShotDetection.POWERSHOT_DIMENSIONS_IN.width / 2)),
                (int)(powerShotCenter.y + (goalUnit * PowerShotDetection.POWERSHOT_DIMENSIONS_IN.height / 2))
        );

        Core.inRange(hsv, RED_LOWER_1, RED_UPPER_1, redMask1);
        Core.inRange(hsv, RED_LOWER_2, RED_UPPER_2, redMask2);
        Core.add(redMask1, redMask2, redMask);
        //Imgproc.erode(redMask, redMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);
        //Imgproc.dilate(redMask, redMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);

        Mat psMask = new Mat(hsv.size(), CvType.CV_8U);
        Imgproc.rectangle(psMask, powerShotTL, powerShotBR, new Scalar(255,255,255), -1);
        Core.bitwise_and(redMask, psMask, psMask);

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(psMask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        powerShots.setContours(CVHelpers.getLargestContours(contours, 3));

        Imgproc.rectangle(input, powerShotTL, powerShotBR, new Scalar(255,255,255), 2);
    }

    public Point getCenterOfLargestContour() {
        if (blue.getArea() > red.getArea()) {
            return blue.getCenter();
        }

        if (red.getArea() > blue.getArea()) {
            return red.getCenter();
        }

        return Detection.INVALID_POINT;
    }

    public double getAreaOfLargestContour() {
        return Math.max(blue.getArea(), red.getArea());
    }

    public Detection getRed() {
        return red;
    }

    public Detection getBlue() {
        return blue;
    }

    public PowerShotDetection getPowerShots() {
        return powerShots;
    }
}
