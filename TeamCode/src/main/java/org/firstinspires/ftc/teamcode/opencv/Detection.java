package org.firstinspires.ftc.teamcode.opencv;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import static org.firstinspires.ftc.teamcode.CVHelpers.drawConvexHull;
import static org.firstinspires.ftc.teamcode.CVHelpers.drawPoint;
import static org.firstinspires.ftc.teamcode.CVHelpers.getBottomLeftOfContour;
import static org.firstinspires.ftc.teamcode.CVHelpers.getCenterOfContour;
import static org.firstinspires.ftc.teamcode.Constants.GREEN;
import static org.firstinspires.ftc.teamcode.Constants.INVALID_AREA;
import static org.firstinspires.ftc.teamcode.Constants.INVALID_POINT;

// Class for a Detection
public class Detection {
    private final double minAreaPx;
    private final Size maxSizePx;
    private double areaPx =  INVALID_AREA;
    private Point centerPx = INVALID_POINT;
    private Point bottomLeftPx = INVALID_POINT;
    private MatOfPoint contour;

    // Constructor
    public Detection(Size maxSize, double minAreaFactor) {
        this.maxSizePx = maxSize;
        this.minAreaPx = maxSize.area() * minAreaFactor;
    }

    // Draw a convex hull around the current detection on the given image
    public void draw(Mat img, Scalar color) {
        if (isValid()) {
            drawConvexHull(img, contour, color);
            drawPoint(img, centerPx, GREEN);
            drawPoint(img, bottomLeftPx, GREEN);
        }
    }

    // Check if the current Detection is valid
    public boolean isValid() {
        return (this.contour != null) && (this.centerPx != INVALID_POINT) && (this.areaPx != INVALID_AREA);
    }

    // Get the current contour
    public MatOfPoint getContour() {
        return contour;
    }

    // Set the values of the current contour
    public void setContour(MatOfPoint contour) {
        this.contour = contour;

        double area;
        if (contour != null && (area = Imgproc.contourArea(contour)) > minAreaPx) {
            this.areaPx = area;
            this.centerPx = getCenterOfContour(contour);
            this.bottomLeftPx = getBottomLeftOfContour(contour);
        } else {
            this.areaPx = INVALID_AREA;
            this.centerPx = INVALID_POINT;
            this.bottomLeftPx = INVALID_POINT;
        }
    }

    // Returns the center of the Detection, normalized so that the width and height of the frame is from [-50,50]
    public Point getCenter() {
        if (!isValid()) {
            return INVALID_POINT;
        }

        double normalizedX = ((centerPx.x / maxSizePx.width) * 100) - 50;
        double normalizedY = ((centerPx.y / maxSizePx.height) * -100) + 50;

        return new Point(normalizedX, normalizedY);
    }

    // Get the center point in pixels
    public Point getCenterPx() {
        return centerPx;
    }

    // Get the area of the Detection, normalized so that the area of the frame is 100
    public double getArea() {
        if (!isValid()) {
            return INVALID_AREA;
        }

        return (areaPx / (maxSizePx.width * maxSizePx.height)) * 100;
    }

    // Get the leftmost bottom corner of the detection
    public Point getBottomLeftCornerPx() {
        return bottomLeftPx;
    }
}