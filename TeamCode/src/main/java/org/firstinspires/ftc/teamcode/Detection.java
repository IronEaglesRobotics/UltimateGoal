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

//Note: how does this differ from CVHelpers? If someone could explain that in a comment here that would be great.
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

    public void draw(Mat img, Scalar color) {
        if (isValid()) {
            drawConvexHull(img, contour, color);
            drawPoint(img, centerPx, GREEN);
        }
    }

    public boolean isValid() {
        return this.contour != null && this.centerPx != INVALID_POINT && this.areaPx != INVALID_AREA;
    }

    public MatOfPoint getContour() {
        return contour;
    }

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

    public Point getCenter() {
        if (!isValid()) {
            return INVALID_POINT;
        }

        double normalizedX = ((centerPx.x / maxSizePx.width) * 100) - 50;
        double normalizedY = ((centerPx.y / maxSizePx.height) * -100) + 50;

        return new Point(normalizedX, normalizedY);
    }

    public double getArea() {
        if (!isValid()) {
            return INVALID_AREA;
        }

        return (areaPx / (maxSizePx.width * maxSizePx.height)) * 100;
    }
}