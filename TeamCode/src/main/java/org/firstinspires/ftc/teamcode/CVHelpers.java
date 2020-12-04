package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.Collections;
import java.util.List;

/*Terms:
* - Contour ... An outline, especially one bounding a shape. In this case, it is whatever OpenCV thinks is a shape.
* - Point ... A point on the image. Note: x=0 and y=0 start in the *top left*.
* - Imgproc ... The core OpenCV instance.
* - Moments ... The average of all grayscale values in an image. Used for finding the center of an image. See: https://www.learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/
* - Mat ... Matrix, what you store images in. Named such because an image is just a matrix of pixels.
* - MatOfInt, MatOfFLoat, MatOf[Type] ... The Mat data type but the cells store data in a certain way.
*/

public class CVHelpers {

    public static final Scalar RED = new Scalar(255, 0, 0);
    public static final Scalar GREEN = new Scalar(0, 255, 0);
    public static final Scalar BLUE = new Scalar(0, 0, 255);
    public static final Scalar WHITE = new Scalar(255, 255, 255);

    public static void drawPoint(Mat img, Point point, Scalar color) {
        Imgproc.circle(img, point, 3, color,  -1);
    }

    public static Point getCenterOfContour(MatOfPoint contour) {
        Moments moments = Imgproc.moments(contour);
        return new Point(moments.m10 / moments.m00, moments.m01/ moments.m00);
    }

    public static void drawContour(Mat img, MatOfPoint contour, Scalar color) {
        Imgproc.drawContours(img, Collections.singletonList(contour), 0, color, 2);
    }

    public static void drawConvexHull(Mat img, MatOfPoint contour, Scalar color) {
        MatOfInt hull =  new MatOfInt();
        Imgproc.convexHull(contour, hull);
        Imgproc.drawContours(img, Collections.singletonList(convertIndexesToPoints(contour, hull)), 0, color, 2);
    }

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

    public static MatOfPoint getLargestContour(List<MatOfPoint> contours) {
        if (contours.size() == 0) {
            return null;
        }

        int largestContourIndex = 0;
        double largestContourArea = 0;
        for (int i = 0; i < contours.size(); i++) {
            double area = Imgproc.contourArea(contours.get(i));
            if (area > largestContourArea) {
                largestContourArea = area;
                largestContourIndex = i;
            }
        }

        return contours.get(largestContourIndex);
    }
}
