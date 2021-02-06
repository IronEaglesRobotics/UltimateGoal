package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.Collections;
import java.util.Comparator;
import java.util.List;

// CV Helper Functions
public class CVHelpers {

    // Draw a point
    public static void drawPoint(Mat img, Point point, Scalar color) {
        Imgproc.circle(img, point, 3, color,  -1);
    }

    // Get the center of a contour
    public static Point getCenterOfContour(MatOfPoint contour) {
        Moments moments = Imgproc.moments(contour);
        return new Point(moments.m10 / moments.m00, moments.m01/ moments.m00);
    }

    // Draw a contour
    public static void drawContour(Mat img, MatOfPoint contour, Scalar color) {
        Imgproc.drawContours(img, Collections.singletonList(contour), 0, color, 2);
    }

    // Draw a convex hull around a contour
    public static void drawConvexHull(Mat img, MatOfPoint contour, Scalar color) {
        MatOfInt hull =  new MatOfInt();
        Imgproc.convexHull(contour, hull);
        Imgproc.drawContours(img, Collections.singletonList(convertIndexesToPoints(contour, hull)), 0, color, 2);
    }

    // Convert indexes to points that is used in order to draw the contours
    public static MatOfPoint convertIndexesToPoints(MatOfPoint contour, MatOfInt indexes) {
        int[] arrIndex = indexes.toArray();
        Point[] arrContour = contour.toArray();
        Point[] arrPoints = new Point[arrIndex.length];

        for (int i=0;i<arrIndex.length;i++) {
            arrPoints[i] = arrContour[arrIndex[i]];
        }

        MatOfPoint hull = new MatOfPoint();
        hull.fromArray(arrPoints);
        return hull;
    }

    // Get the largest contour out of a list
    public static MatOfPoint getLargestContour(List<MatOfPoint> contours) {
        if (contours.size() == 0) {
            return null;
        }
        return getLargestContours(contours, 1).get(0);
    }

    // Get the top largest contours
    public static List<MatOfPoint> getLargestContours(List<MatOfPoint> contours, int numContours) {
        Collections.sort(contours, new Comparator<MatOfPoint>() {
            @Override
            public int compare(MatOfPoint a, MatOfPoint b) {
                return (int) Imgproc.contourArea(b) - (int) Imgproc.contourArea(a);
            }
        });
        return contours.subList(0, Math.min(numContours, contours.size()));
    }
}
