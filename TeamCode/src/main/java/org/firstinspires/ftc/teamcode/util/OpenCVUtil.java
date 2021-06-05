package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.util.enums.Alliance;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.Collections;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.teamcode.util.Configurables.CAMERA_BLUE_GOAL_LOWER;
import static org.firstinspires.ftc.teamcode.util.Configurables.CAMERA_BLUE_GOAL_UPPER;
import static org.firstinspires.ftc.teamcode.util.Configurables.CAMERA_RED_GOAL_LOWER;
import static org.firstinspires.ftc.teamcode.util.Configurables.CAMERA_RED_GOAL_UPPER;
import static org.firstinspires.ftc.teamcode.util.Configurables.CV_GOAL_ALLOWABLE_AREA_ERROR;
import static org.firstinspires.ftc.teamcode.util.Configurables.CV_GOAL_ALLOWABLE_ASPECT_ERROR;
import static org.firstinspires.ftc.teamcode.util.Configurables.CV_GOAL_ALLOWABLE_SIZE_ERROR;
import static org.firstinspires.ftc.teamcode.util.Configurables.CV_GOAL_ALLOWABLE_SOLIDARITY_ERROR;
import static org.firstinspires.ftc.teamcode.util.Configurables.CV_GOAL_ALLOWABLE_Y_ERROR;
import static org.firstinspires.ftc.teamcode.util.Configurables.CV_GOAL_ASPECT_RATIO;
import static org.firstinspires.ftc.teamcode.util.Configurables.CV_GOAL_PROPER_AREA;
import static org.firstinspires.ftc.teamcode.util.Configurables.CV_GOAL_PROPER_ASPECT;
import static org.firstinspires.ftc.teamcode.util.Configurables.CV_GOAL_PROPER_HEIGHT;
import static org.firstinspires.ftc.teamcode.util.Configurables.CV_MAX_GOAL_AREA;
import static org.firstinspires.ftc.teamcode.util.Configurables.CV_MIN_GOAL_AREA;

// CV Helper Functions
@Config
public class OpenCVUtil {

    public static String telem = "nothing";

    // Draw a point
    public static void drawPoint(Mat img, Point point, Scalar color) {
        Imgproc.circle(img, point, 3, color,  -1);
    }

    // Get the center of a contour
    public static Point getCenterOfContour(MatOfPoint contour) {
        Moments moments = Imgproc.moments(contour);
        return new Point(moments.m10 / moments.m00, moments.m01/ moments.m00);
    }

    // Get the bottom left of a contour
    public static Point getBottomLeftOfContour(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return new Point(boundingRect.x, boundingRect.y+boundingRect.height);
    }

    // Get the bottom right of a contour
    public static Point getBottomRightOfContour(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return new Point(boundingRect.x+boundingRect.width, boundingRect.y+boundingRect.height);
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

    // Draw a filled in convex hull around a contour
    public static void fillConvexHull(Mat img, MatOfPoint contour, Scalar color) {
        MatOfInt hull =  new MatOfInt();
        Imgproc.convexHull(contour, hull);
        Imgproc.drawContours(img, Collections.singletonList(convertIndexesToPoints(contour, hull)), 0, color, -1);
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
        Collections.sort(contours, (a, b) -> (int) Imgproc.contourArea(b) - (int) Imgproc.contourArea(a));
        return contours.subList(0, Math.min(numContours, contours.size()));
    }

    public static MatOfPoint getHighGoalContour(List<MatOfPoint> contours) {
        Collections.sort(contours, (a, b) -> (int) Imgproc.contourArea(b) - (int) Imgproc.contourArea(a));
        // return null if nothing
        if (contours.size() == 0) {
            return null;
        }
        // check each contour for touching the top, aspect, and size
        double properAspect = ((double)CV_GOAL_ASPECT_RATIO.height) / ((double)CV_GOAL_ASPECT_RATIO.width);
        for (int i = 0; i < contours.size() - 1; i++) {
            MatOfPoint contour = contours.get(i);
            Rect rect = Imgproc.boundingRect(contour);
            double area = Imgproc.contourArea(contour);
            double aspect = ((double) rect.height) / ((double) rect.width);
            if (rect.y <= -100 || Math.abs(properAspect - aspect) > CV_GOAL_ALLOWABLE_ASPECT_ERROR ||
                    area < CV_MIN_GOAL_AREA || area > CV_MAX_GOAL_AREA) {
                contours.remove(i);
                i--;
            }
        }
        // check for 2 that can be combined
        int goalCounter = -1;
        for (int i = 0; i < contours.size() - 1; i++) {
            MatOfPoint contour1 = contours.get(i);
            MatOfPoint contour2 = contours.get(i + 1);
            Rect rect1 = Imgproc.boundingRect(contour1);
            Rect rect2 = Imgproc.boundingRect(contour2);
            double area1 = Imgproc.contourArea(contour1);
            double area2 = Imgproc.contourArea(contour2);
            if (Math.abs(Math.abs(rect1.y) - Math.abs(rect2.y)) < CV_GOAL_ALLOWABLE_Y_ERROR &&
                    Math.abs(area1 - area2) < CV_GOAL_ALLOWABLE_SIZE_ERROR) {
                goalCounter = i;
                break;
            }
        }
        // return the results
        if (goalCounter == -1) {
            return contours.get(0);
        } else {
            MatOfPoint highGoal = new MatOfPoint();
            highGoal.push_back(contours.get(goalCounter));
            highGoal.push_back(contours.get(goalCounter + 1));
            return highGoal;
        }
    }

    public static MatOfPoint getConfidenceContour(List<MatOfPoint> contours, Mat frame, Mat telemFrame, Alliance alliance) {
        if (contours.size() == 0 || frame == null) {
            return null;
        }

        double highestConfidence = 0;
        int highestConfidenceCounter = 0;
        for (int i = 0; i < contours.size(); i++) {
            MatOfPoint contour = contours.get(i);

            double area = (Imgproc.contourArea(contour) / (frame.width()*frame.height()) ) * 100;
            Rect rect = Imgproc.boundingRect(contour);

            // area check
            double difference = Math.abs(CV_GOAL_PROPER_AREA - area);

            double areaConfidence;
            if (difference <= CV_GOAL_ALLOWABLE_AREA_ERROR) {
                areaConfidence = 1;
            } else {
                areaConfidence = Math.min(area, CV_GOAL_PROPER_AREA)/Math.max(area, CV_GOAL_PROPER_AREA);
            }

            // aspect check
            double properAspectRatio = ((double)CV_GOAL_PROPER_ASPECT.height)/((double)CV_GOAL_PROPER_ASPECT.width);
            double wantedAspectRatio = ((double) rect.height) / ((double) rect.width);
            double aspectConfidence = 1 - Math.abs(properAspectRatio - wantedAspectRatio) / properAspectRatio;

            // solidarity check
            double error = Math.abs(area - rect.area());
            double solidarityConfidence;
            if (error <= CV_GOAL_ALLOWABLE_SOLIDARITY_ERROR) {
                solidarityConfidence = 1;
            } else {
                solidarityConfidence = area / ((rect.area() / (frame.width()*frame.height())) * 100);
            }

            // side colors check
            double inch = rect.width / 11.0;
            int left = Math.max(0, (int) (rect.x - (3 * inch)));
            left = Math.min(left, frame.width());
            int right = Math.max(0, (int) (rect.x + rect.width +  (3 * inch)));
            right = Math.min(right, frame.width());
            int yValue = Math.max((int)(rect.y + (rect.height / 2.0)), 0);
            yValue = Math.min(yValue, frame.height());

            double[] leftColor = frame.get(yValue, left);
            double[] rightColor = frame.get(yValue, right);

            double sideCheckConfidence = 0;
            if (alliance == Alliance.RED) {
                if (leftColor != null && rightColor != null) {
                    if ((leftColor[0] < CAMERA_RED_GOAL_UPPER.h || leftColor[0] > CAMERA_RED_GOAL_LOWER.h) &&
                            (CAMERA_RED_GOAL_LOWER.s < leftColor[1] && leftColor[1] < CAMERA_RED_GOAL_UPPER.s) &&
                            (CAMERA_RED_GOAL_LOWER.v < leftColor[2] && leftColor[2] < CAMERA_RED_GOAL_UPPER.v)) {
                        sideCheckConfidence += 1;
                    }
                    if ((rightColor[0] < CAMERA_RED_GOAL_UPPER.h || rightColor[0] > CAMERA_RED_GOAL_LOWER.h) &&
                            (CAMERA_RED_GOAL_LOWER.s < rightColor[1] && rightColor[1] < CAMERA_RED_GOAL_UPPER.s) &&
                            (CAMERA_RED_GOAL_LOWER.v < rightColor[2] && rightColor[2] < CAMERA_RED_GOAL_UPPER.v)) {
                        sideCheckConfidence += 1;
                    }
                }
            } else if (alliance == Alliance.BLUE) {
                if (leftColor != null && rightColor != null) {
                    if ((CAMERA_BLUE_GOAL_LOWER.h < leftColor[0] && leftColor[0] < CAMERA_BLUE_GOAL_UPPER.h) &&
                            (CAMERA_BLUE_GOAL_LOWER.s < leftColor[1] && leftColor[1] < CAMERA_BLUE_GOAL_UPPER.s) &&
                            (CAMERA_BLUE_GOAL_LOWER.v < leftColor[2] && leftColor[2] < CAMERA_BLUE_GOAL_UPPER.v)) {
                        sideCheckConfidence += 1;
                    }
                    if ((CAMERA_BLUE_GOAL_LOWER.h < rightColor[0] && rightColor[0] < CAMERA_BLUE_GOAL_UPPER.h) &&
                            (CAMERA_BLUE_GOAL_LOWER.s < rightColor[1] && rightColor[1] < CAMERA_BLUE_GOAL_UPPER.s) &&
                            (CAMERA_BLUE_GOAL_LOWER.v < rightColor[2] && rightColor[2] < CAMERA_BLUE_GOAL_UPPER.v)) {
                        sideCheckConfidence += 1;
                    }
                }
            }

            // height check
            double heightConfidence = 1 - (Math.abs(CV_GOAL_PROPER_HEIGHT - (rect.y+(rect.height/2.0))) / (frame.height() - CV_GOAL_PROPER_HEIGHT));

            double totalConfidence = areaConfidence + aspectConfidence + solidarityConfidence + sideCheckConfidence + heightConfidence;

            if (totalConfidence > highestConfidence) {
                highestConfidenceCounter = i;
                highestConfidence = totalConfidence;
                if (leftColor != null && rightColor != null) {
                    telem = String.format(Locale.US, "Area Confidence: %s\nAspect Confidence: %s\nSolidarity Confidence: %s\nSide Check Confidence: %s\nHeight Confidence: %s\nH: %s S: %s V: %s\nH: %s S: %s V: %s", areaConfidence, aspectConfidence, solidarityConfidence, sideCheckConfidence, heightConfidence, leftColor[0], leftColor[1], leftColor[2], rightColor[0], rightColor[1], rightColor[2]);
                } else {
                    telem = String.format(Locale.US, "Area Confidence: %s\nAspect Confidence: %s\nSolidarity Confidence: %s\nSide Check Confidence: %s\nHeight Confidence: %s", areaConfidence, aspectConfidence, solidarityConfidence, sideCheckConfidence, heightConfidence);
                }
            }
        }

        return contours.get(highestConfidenceCounter);
    }
}
