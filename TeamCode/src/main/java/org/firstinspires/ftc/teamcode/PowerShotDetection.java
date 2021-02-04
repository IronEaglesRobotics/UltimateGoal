package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class PowerShotDetection {
    public static final Size GOAL_DIMENSIONS_IN = new Size(24, 15.5);
    public static final Point POWERSHOT_OFFSET_IN = new Point(-29, -11);
    public static final Size POWERSHOT_DIMENSIONS_IN = new Size(30, 25);

    private ArrayList<Detection> powerShots;

    //==Constructor==//
    //Makes an arraylist that holds all the things that look like powershots within the specified parameters.
    PowerShotDetection(final Size maxSize, final double minAreaFactor) {
        powerShots = new ArrayList<Detection>() {{
            add(new Detection(maxSize, minAreaFactor));
            add(new Detection(maxSize, minAreaFactor));
            add(new Detection(maxSize, minAreaFactor));
        }};
    }

    //Counts valid powershots.
    public int getCount() {
        int count = 0;
        for (Detection detection : powerShots) {
            if (detection.isValid()) {
                count++;
            }
        }
        return count;
    }

    //Gets powerShot of specified number.
    public Detection get(int i) {
        return powerShots.get(i);
    }

    //Sorts out the detected contours and sets `powerShots` accordingly.
    public void setContours(List<MatOfPoint> contours) {
        //Sort detected contours by x values, presumably left to right.
        Collections.sort(contours, new Comparator<MatOfPoint>() {
            @Override
            public int compare(MatOfPoint a, MatOfPoint b) {
                return (int)CVHelpers.getCenterOfContour(a).x - (int)CVHelpers.getCenterOfContour(b).x;
            }
        });

        for (int i = 0; i < 3; i++) {
            if (i >= contours.size()) {
                powerShots.get(i).setContour(null);
            } else {
                powerShots.get(i).setContour(contours.get(i));
            }
        }
    }

    //Draw all valid contours.
    public void draw(Mat img, Scalar color) {
        for (Detection detection: powerShots) {
            MatOfPoint contour = detection.getContour();
            if (detection.isValid()) {
                CVHelpers.drawContour(img, contour, color);
            }
        }
    }
}