package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Camera {
    private OpenCvCamera stackCamera;
    private OpenCvCamera targetingCamera;
    private StarterStackPipeline stackPipeline;
    private TargetingPipeline targetingPipeline;
    private int stackCameraMonitorViewId;
    private int targetingCameraMonitorViewId;

    private boolean stackCameraInitialized;
    private boolean targetingCameraInitialized;

    private final double SINGLE_MIN_AREA = 0.01;
    private final double QUAD_MIN_AREA = 0.1;

    public Camera(HardwareMap hardwareMap) {
        stackCameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.stackCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Stack Webcam"), stackCameraMonitorViewId);
        this.stackPipeline = new StarterStackPipeline();
        stackCamera.setPipeline(stackPipeline);

        targetingCameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.stackCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Targeting Webcam"), targetingCameraMonitorViewId);
        this.targetingPipeline = new TargetingPipeline();
        targetingCamera.setPipeline(targetingPipeline);
    }

    public void initStackCamera() {
        stackCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                stackCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });
        stackCameraInitialized = true;
    }

    public void stopStackCamera() {

    }

    public void initTargetingCamera() {
        targetingCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                targetingCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });
        targetingCameraInitialized = false;
    }

    public void stopTargetingCamera() {

    }

    public StarterStack checkStack() {
        if (stackCameraInitialized) {
            if (stackPipeline.getStarterStackArea() > QUAD_MIN_AREA) {
                return StarterStack.QUAD;
            } else if (stackPipeline.getStarterStackArea() > SINGLE_MIN_AREA) {
                return StarterStack.SINGLE;
            } else {
                return StarterStack.NONE;
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

    public PowerShotDetection getPowerShots() {
        return targetingPipeline.getPowerShots();
    }
}
