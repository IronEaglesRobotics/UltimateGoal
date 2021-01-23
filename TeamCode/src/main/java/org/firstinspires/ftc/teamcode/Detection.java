package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import static org.firstinspires.ftc.teamcode.CVHelpers.GREEN;
import static org.firstinspires.ftc.teamcode.CVHelpers.drawConvexHull;
import static org.firstinspires.ftc.teamcode.CVHelpers.drawPoint;
import static org.firstinspires.ftc.teamcode.CVHelpers.getCenterOfContour;

//Not entirely sure what this does, but I'm pretty sure this is a general target detection class since it is used in every single CV class.
public class Detection {
    public static final Point INVALID_POINT = new Point(Double.MIN_VALUE, Double.MIN_VALUE);
    public static final double INVALID_AREA = -1;

    private final double minAreaPx;
    private final Size maxSizePx;

    private double areaPx =  INVALID_AREA;
    private Point centerPx = INVALID_POINT;
    private MatOfPoint contour;

    //==Constructor==//
    Detection(Size maxSize, double minAreaFactor) {
        this.maxSizePx = maxSize;
        this.minAreaPx = maxSize.area() * minAreaFactor;
    }

    //Draw a convex hull (see tutorial if confused) of a certain color around the image contours. Then draw a green point at centerPx.
    //https://www.learnopencv.com/convex-hull-using-opencv-in-python-and-c/
    public void draw(Mat img, Scalar color) {
        if (isValid()) {
            drawConvexHull(img, contour, color);
            drawPoint(img, centerPx, GREEN);
        }
    }

    //After you initialize the class, the isValid() function makes sure you have a contour, a valid center point, and a valid area.
    //Note: added parenthesis according to the chart to make Sr. Scott's code more readable.
    //https://introcs.cs.princeton.edu/java/11precedence/
    public boolean isValid() {
        return (this.contour != null) && (this.centerPx != INVALID_POINT) && (this.areaPx != INVALID_AREA);
    }

    public MatOfPoint getContour() {
        return contour;
    }

    //Have somebody take a look at this, I have no idea what is going on.
    public void setContour(MatOfPoint contour) {
        this.contour = contour;

        double area;
        if (contour != null && (area = Imgproc.contourArea(contour)) > minAreaPx) {
            this.areaPx = area;
            this.centerPx = getCenterOfContour(contour);
        } else {
            this.areaPx = INVALID_AREA;
            this.centerPx = INVALID_POINT;
        }
    }

    //Somebody needs to be clear about what normalization means in this case.
    public Point getCenter() {

        if (!isValid()) {
            return INVALID_POINT;
        }

        double normalizedX = ((centerPx.x / maxSizePx.width) * 100) - 50;
        double normalizedY = ((centerPx.y / maxSizePx.height) * -100) + 50;

        return new Point(normalizedX, normalizedY);
    }

    public Point getCenterPx() {
        return centerPx;
    }

    //Shouldn't this just get the regular area in pixels?
    public double getArea() {
        if (!isValid()) {
            return INVALID_AREA;
        }

        return (areaPx / (maxSizePx.width * maxSizePx.height)) * 100;
    }
}