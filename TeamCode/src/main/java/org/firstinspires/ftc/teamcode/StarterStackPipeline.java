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
import static org.firstinspires.ftc.teamcode.CVHelpers.WHITE;
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

class StarterStackPipeline extends OpenCvPipeline
{
    Mat blurred = new Mat();
    Mat hsv = new Mat();
    Mat orangeMask = new Mat();

    final Mat STRUCTURING_ELEMENT = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size((2 * 2) + 1, (2 * 2) + 1));
    final Point ANCHOR = new Point((STRUCTURING_ELEMENT.cols() / 2f), STRUCTURING_ELEMENT.rows() / 2f);
    final double MIN_STARTERSTACK_AREA = 0.005;
    final Scalar ORANGE_LOWER = new Scalar(10, 85, 100);
    final Scalar ORANGE_UPPER = new Scalar(25, 255, 255);
    final int ERODE_DILATE_ITERATIONS = 2;
    final Size BLUR_SIZE = new Size(11, 11);

    private Detection starterStack;

    @Override
    public void init(Mat input) {
        starterStack = new Detection(input.size(), MIN_STARTERSTACK_AREA);
    }

    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.GaussianBlur(input, blurred, BLUR_SIZE, 0);
        Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);

        updateStarterStack(input);
        starterStack.draw(input, WHITE);

        return input;
    }

    private void updateStarterStack(Mat input) {
        Core.inRange(hsv, ORANGE_LOWER, ORANGE_UPPER, orangeMask);

        //Erode and then dilate the mask to get rid of white noise.
        //This can also be done via. Opening. See:
        //https://docs.opencv.org/master/d9/d61/tutorial_py_morphological_ops.html
        Imgproc.erode(orangeMask, orangeMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);
        Imgproc.dilate(orangeMask, orangeMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);

        Rect starterStackBox = new Rect(0,75,input.cols()/2,90);
        Mat starterStackMask = new Mat(hsv.size(), CvType.CV_8U);//320 240

        Imgproc.rectangle(starterStackMask, starterStackBox, new Scalar(255,255,255), -1);
        Core.bitwise_and(orangeMask, starterStackMask, starterStackMask);

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(starterStackMask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        starterStack.setContour(CVHelpers.getLargestContour(contours));

        Imgproc.rectangle(input, starterStackBox, new Scalar(255,255,255), 2);
    }

    public double getStarterStackArea() {
        return starterStack.getArea();
    }

    public Detection getStarterStack() {
        return starterStack;
    }
}
