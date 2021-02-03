package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Camera {
    private HardwareMap hardwareMap;
    private OpenCvCamera stackCamera;
    private OpenCvCamera targetingCamera;
    private StarterStackPipeline stackPipeline;
    private TargetingPipeline targetingPipeline;
    private int stackCameraMonitorViewId;
    private int targetingCameraMonitorViewId;

    private boolean stackCameraInitialized;
    private boolean targetingCameraInitialized;

    private final double SINGLE_MIN_AREA = 0.7;
    private final double QUAD_MIN_AREA = 1.85;

    public Camera(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void initStackCamera() {
        stackCameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.stackCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Stack Webcam"), stackCameraMonitorViewId);
        this.stackPipeline = new StarterStackPipeline();
        stackCamera.setPipeline(stackPipeline);
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
        stackCamera.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener()
        {
            @Override
            public void onClose() {}
        });
        stackCameraInitialized = false;
    }

    public void initTargetingCamera() {
        targetingCameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.targetingCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Targeting Webcam"), targetingCameraMonitorViewId);
        this.targetingPipeline = new TargetingPipeline();
        targetingCamera.setPipeline(targetingPipeline);
        targetingCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                targetingCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
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

    public PowerShotDetection getPowerShots() {
        return targetingPipeline.getPowerShots();
    }

    public double getSize() {
        if (stackPipeline.getStarterStack() != null) {
            return stackPipeline.getStarterStackArea();
        }
        return 0;
    }
}
