package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.Detection;
import org.firstinspires.ftc.teamcode.vision.PowershotDetection;
import org.firstinspires.ftc.teamcode.vision.StarterStackPipeline;
import org.firstinspires.ftc.teamcode.vision.TargetingPipeline;
import org.firstinspires.ftc.teamcode.util.enums.StarterStack;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.util.Constants.INVALID_DETECTION;
import static org.firstinspires.ftc.teamcode.util.Constants.INVALID_POWERSHOT_DETECTION;
import static org.firstinspires.ftc.teamcode.util.Constants.MIN_STARTERSTACK_QUAD_AREA;
import static org.firstinspires.ftc.teamcode.util.Constants.MIN_STARTERSTACK_SINGLE_AREA;
import static org.firstinspires.ftc.teamcode.util.Constants.STACK_WEBCAM;
import static org.firstinspires.ftc.teamcode.util.Constants.TARGETING_WEBCAM;
import static org.firstinspires.ftc.teamcode.util.Constants.WEBCAM_HEIGHT;
import static org.firstinspires.ftc.teamcode.util.Constants.WEBCAM_ROTATION;
import static org.firstinspires.ftc.teamcode.util.Constants.WEBCAM_WIDTH;
import static org.firstinspires.ftc.teamcode.util.enums.StarterStack.NONE;
import static org.firstinspires.ftc.teamcode.util.enums.StarterStack.QUAD;
import static org.firstinspires.ftc.teamcode.util.enums.StarterStack.SINGLE;

// Class for the camera
public class Camera {
    private HardwareMap hardwareMap;
    private OpenCvCamera stackCamera;
    private OpenCvCamera targetingCamera;
    private StarterStackPipeline stackPipeline;
    private TargetingPipeline targetingPipeline;

    private boolean stackCameraInitialized;
    private boolean targetingCameraInitialized;

    // Constructor
    public Camera(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    // Initiate the StarterStack Camera
    public void initStackCamera() {
        int stackCameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.stackCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, STACK_WEBCAM), stackCameraMonitorViewId);
        this.stackPipeline = new StarterStackPipeline();
        stackCamera.setPipeline(stackPipeline);
        stackCamera.openCameraDeviceAsync(() -> {
            stackCamera.startStreaming(WEBCAM_WIDTH, WEBCAM_HEIGHT, WEBCAM_ROTATION);
            stackCameraInitialized = true;
        });
    }

    // Close the StarterStack Camera
    public void stopStackCamera() {
        if (stackCameraInitialized) {
            stackCamera.closeCameraDeviceAsync(() -> stackCameraInitialized = false);
        }
    }

    // Initiate the Targeting Camera
    public void initTargetingCamera() {
        int targetingCameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.targetingCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, TARGETING_WEBCAM), targetingCameraMonitorViewId);
        this.targetingPipeline = new TargetingPipeline();
        targetingCamera.setPipeline(targetingPipeline);
        targetingCamera.openCameraDeviceAsync(() -> {
            targetingCamera.startStreaming(WEBCAM_WIDTH, WEBCAM_HEIGHT, WEBCAM_ROTATION);
            targetingCameraInitialized = true;
        });
    }

    // Close the Targeting Camera
    public void stopTargetingCamera() {
        if (targetingCameraInitialized) {
            targetingCamera.closeCameraDeviceAsync(() -> targetingCameraInitialized = false);
        }
    }

    // Check what StarterStack configuration is on the field
    public StarterStack checkStack() {
        if (stackCameraInitialized) {
            double area = stackPipeline.getStarterStack().getArea();
            if (area > MIN_STARTERSTACK_QUAD_AREA) {
                return QUAD;
            } else if (area > MIN_STARTERSTACK_SINGLE_AREA){
                return SINGLE;
            }
        }
        return NONE;
    }

    // Get the StarterStack Detection
    public Detection getStarterStack() {
        return (stackCameraInitialized ? stackPipeline.getStarterStack() : INVALID_DETECTION);
    }

    // Get the Red Goal Detection
    public Detection getRed() {
        return (targetingCameraInitialized ? targetingPipeline.getRed() : INVALID_DETECTION);
    }

    // Get the Blue Goal Detection
    public Detection getBlue() {
        return (targetingCameraInitialized ? targetingPipeline.getBlue() : INVALID_DETECTION);
    }

    // Get the Red Powershot Detection
    public PowershotDetection getRedPowershots() {
        return targetingCameraInitialized ? targetingPipeline.getRedPowershots() : INVALID_POWERSHOT_DETECTION;
    }

    // Get the Blue Powershot Detection
    public PowershotDetection getBluePowershots() {
        return targetingCameraInitialized ? targetingPipeline.getBluePowershots() : INVALID_POWERSHOT_DETECTION;
    }

    // Get the number of frames the current camera has processed
    public int getFrameCount() {
        if (stackCameraInitialized) {
            return stackCamera.getFrameCount();
        } else if (targetingCameraInitialized) {
            return targetingCamera.getFrameCount();
        } else {
            return 0;
        }
    }

    // Get Telemetry for the current active camera
    public String getTelemetry() {
        if (stackCameraInitialized) {
            return String.format(Locale.US, "Stack: %s\nSize: %.4f", checkStack(), getStarterStack().getArea());
        } else if (targetingCameraInitialized) {
            return String.format(Locale.US, "Red Goal:  Area: %.2f Center: (%.2f,%.2f)\n" +
                                                    "Blue Goal: Area: %.2f Center: (%.2f,%.2f)" +
                                                    "Red PowerShots: (%.1f,%.1f) (%.1f,%.1f) (%.1f,%.1f)\n" +
                                                    "BluePowerShots: (%.1f,%.1f) (%.1f,%.1f) (%.1f,%.1f)\n",
                    targetingPipeline.getRed().getArea(), targetingPipeline.getRed().getCenter().x, targetingPipeline.getRed().getCenter().x,
                    targetingPipeline.getBlue().getArea(), targetingPipeline.getBlue().getCenter().x, targetingPipeline.getBlue().getCenter().x,
                    targetingPipeline.getRedPowershots().get(0).getCenter().x, targetingPipeline.getRedPowershots().get(0).getCenter().y,
                    targetingPipeline.getRedPowershots().get(1).getCenter().x, targetingPipeline.getRedPowershots().get(1).getCenter().y,
                    targetingPipeline.getRedPowershots().get(2).getCenter().x, targetingPipeline.getRedPowershots().get(2).getCenter().y,
                    targetingPipeline.getBluePowershots().get(0).getCenter().x, targetingPipeline.getBluePowershots().get(0).getCenter().y,
                    targetingPipeline.getBluePowershots().get(1).getCenter().x, targetingPipeline.getBluePowershots().get(1).getCenter().y,
                    targetingPipeline.getBluePowershots().get(2).getCenter().x, targetingPipeline.getBluePowershots().get(2).getCenter().y);
        }
        return ("No Camera Initialized");
    }
}