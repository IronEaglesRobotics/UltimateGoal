package org.firstinspires.ftc.teamcode.opencv;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

// Class for the Powershot Detection
public class PowershotDetection {
    private ArrayList<Detection> powershots;

    // Constructor
    public PowershotDetection(final Size maxSize, final double minAreaFactor) {
        powershots = new ArrayList<Detection>() {{
            add(new Detection(maxSize, minAreaFactor));
            add(new Detection(maxSize, minAreaFactor));
            add(new Detection(maxSize, minAreaFactor));
        }};
    }

    // Get the count of the valid powershot detections
    public int getCount() {
        int count = 0;
        for (Detection detection : powershots) {
            if (detection.isValid()) {
                count++;
            }
        }
        return count;
    }

    // Gets powerShot of specified number.
    public Detection get(int i) {
        return powershots.get(i);
    }

    // Get the left most powershot detection
    public Detection getLeftMost() {
        if (powershots.get(0).isValid()) {
            return powershots.get(0);
        } else if (powershots.get(1).isValid()) {
            return powershots.get(1);
        } else if (powershots.get(2).isValid()) {
            return powershots.get(2);
        }
        return powershots.get(0);
    }

    // Sorts out the detected contours and sets `powershots` accordingly.
    public void setContours(List<MatOfPoint> contours) {
        //Sort detected contours by x values, presumably left to right.
        Collections.sort(contours, (a, b) -> (int) CVHelpers.getCenterOfContour(a).x - (int) CVHelpers.getCenterOfContour(b).x);

        for (int i = 0; i < 3; i++) {
            if (i >= contours.size()) {
                powershots.get(i).setContour(null);
            } else {
                powershots.get(i).setContour(contours.get(i));
            }
        }
    }

    // Draw all valid contours.
    public void draw(Mat img, Scalar color) {
        for (Detection detection: powershots) {
            MatOfPoint contour = detection.getContour();
            if (detection.isValid()) {
                CVHelpers.drawContour(img, contour, color);
            }
        }
    }
}