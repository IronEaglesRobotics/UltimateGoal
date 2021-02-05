package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Locale;

public class Camera {
    private HardwareMap hardwareMap;
    private OpenCvCamera stackCamera;
    private OpenCvCamera targetingCamera;
    private StarterStackPipeline stackPipeline;
    private TargetingPipeline targetingPipeline;

    private boolean stackCameraInitialized;
    private boolean targetingCameraInitialized;

    private final int WEBCAM_WIDTH = 320;
    private final int WEBCAM_HEIGHT = 240;
    private final double SINGLE_MIN_AREA = 0.7;
    private final double QUAD_MIN_AREA = 1.85;

    public Camera(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void initStackCamera() {
        int stackCameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.stackCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Stack Webcam"), stackCameraMonitorViewId);
        this.stackPipeline = new StarterStackPipeline();
        stackCamera.setPipeline(stackPipeline);
        stackCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                stackCamera.startStreaming(WEBCAM_WIDTH, WEBCAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }
        });
        stackCameraInitialized = true;
    }

    public void stopStackCamera() {
        stackCamera.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener()
        {
            @Override
            public void onClose() {}
        });
        stackCameraInitialized = false;
    }

    public void initTargetingCamera() {
        int targetingCameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.targetingCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Targeting Webcam"), targetingCameraMonitorViewId);
        this.targetingPipeline = new TargetingPipeline();
        targetingCamera.setPipeline(targetingPipeline);
        targetingCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                targetingCamera.startStreaming(WEBCAM_WIDTH, WEBCAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }
        });
        targetingCameraInitialized = true;
    }

    public void stopTargetingCamera() {
        targetingCamera.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener()
        {
            @Override
            public void onClose() {}
        });
        targetingCameraInitialized = false;
    }

    public StarterStack checkStack() {
        if (stackCameraInitialized && stackPipeline.getStarterStack() != null) {
            double area = stackPipeline.getStarterStackArea();
            if (area > QUAD_MIN_AREA) {
                return StarterStack.QUAD;
            } else if (area > SINGLE_MIN_AREA){
                return StarterStack.SINGLE;
            }
        }
        return StarterStack.NONE;
    }

    public enum StarterStack {
        NONE, SINGLE, QUAD
    }

    public Detection getRed() {
        return targetingPipeline.getRed();
    }

    public Detection getBlue() {
        return targetingPipeline.getBlue();
    }

    public PowerShotDetection getPowerShot() {
        return targetingPipeline.getPowerShot();
    }

    public double getSize() {
        if (stackPipeline.getStarterStack() != null) {
            return stackPipeline.getStarterStackArea();
        }
        return 0;
    }

    public int getFrameCount() {
        if (stackCameraInitialized) {
            return stackCamera.getFrameCount();
        } else if (targetingCameraInitialized) {
            return targetingCamera.getFrameCount();
        } else {
            return 0;
        }
    }

    public String getTelemetry() {
        if (stackCameraInitialized) {
            return String.format(Locale.US, "Stack: %s", checkStack());
        } else if (targetingCameraInitialized) {
            return String.format(Locale.US, "PowerShots: (%.1f,%.1f) (%.1f,%.1f) (%.1f,%.1f)\n" +
                                                    "Red Goal:  Area: %.2f Center: (%.2f,%.2f)\n" +
                                                    "Blue Goal: Area: %.2f Center: (%.2f,%.2f)",
                    targetingPipeline.getPowerShot().get(0).getCenter().x, targetingPipeline.getPowerShot().get(0).getCenter().y,
                    targetingPipeline.getPowerShot().get(1).getCenter().x, targetingPipeline.getPowerShot().get(1).getCenter().y,
                    targetingPipeline.getPowerShot().get(2).getCenter().x, targetingPipeline.getPowerShot().get(2).getCenter().y,
                    targetingPipeline.getRed().getArea(), targetingPipeline.getRed().getCenter().x, targetingPipeline.getRed().getCenter().x,
                    targetingPipeline.getBlue().getArea(), targetingPipeline.getBlue().getCenter().x, targetingPipeline.getBlue().getCenter().x);
        }
        return ("No Camera Initialized");
    }
}