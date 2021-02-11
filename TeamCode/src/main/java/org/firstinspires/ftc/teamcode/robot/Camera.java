package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.opencv.Detection;
import org.firstinspires.ftc.teamcode.opencv.PowershotDetection;
import org.firstinspires.ftc.teamcode.opencv.StarterStackPipeline;
import org.firstinspires.ftc.teamcode.opencv.TargetingPipeline;
import org.firstinspires.ftc.teamcode.opencv.WobbleGoalPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.Constants.INVALID_DETECTION;
import static org.firstinspires.ftc.teamcode.Constants.INVALID_POWERSHOT_DETECTION;
import static org.firstinspires.ftc.teamcode.Constants.MIN_STARTERSTACK_QUAD_AREA;
import static org.firstinspires.ftc.teamcode.Constants.MIN_STARTERSTACK_SINGLE_AREA;
import static org.firstinspires.ftc.teamcode.Constants.STACK_WEBCAM;
import static org.firstinspires.ftc.teamcode.Constants.TARGETING_WEBCAM;
import static org.firstinspires.ftc.teamcode.Constants.WEBCAM_HEIGHT;
import static org.firstinspires.ftc.teamcode.Constants.WEBCAM_ROTATION;
import static org.firstinspires.ftc.teamcode.Constants.WEBCAM_WIDTH;

// Class for the camera
public class Camera {
    private HardwareMap hardwareMap;
    private OpenCvCamera stackCamera;
    private OpenCvCamera targetingCamera;
    private OpenCvCamera wobbleGoalCamera;
    private StarterStackPipeline stackPipeline;
    private TargetingPipeline targetingPipeline;
    private WobbleGoalPipeline wobbleGoalPipeline;

    private boolean stackCameraInitialized;
    private boolean targetingCameraInitialized;
    private boolean wobbleGoalCameraInitialized;

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
        stackCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                stackCamera.startStreaming(WEBCAM_WIDTH, WEBCAM_HEIGHT, WEBCAM_ROTATION);
            }
        });
        stackCameraInitialized = true;
    }

    // Close the StarterStack Camera
    public void stopStackCamera() {
        stackCamera.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener()
        {
            @Override
            public void onClose() {}
        });
        stackCameraInitialized = false;
    }

    // Initiate the Targeting Camera
    public void initTargetingCamera() {
        int targetingCameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.targetingCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, TARGETING_WEBCAM), targetingCameraMonitorViewId);
        this.targetingPipeline = new TargetingPipeline();
        targetingCamera.setPipeline(targetingPipeline);
        targetingCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                targetingCamera.startStreaming(WEBCAM_WIDTH, WEBCAM_HEIGHT, WEBCAM_ROTATION);
            }
        });
        targetingCameraInitialized = true;
    }

    // Close the Targeting Camera
    public void stopTargetingCamera() {
        targetingCamera.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener()
        {
            @Override
            public void onClose() {}
        });
        targetingCameraInitialized = false;
    }

    // Initiate the Wobble Goal Camera
    public void initWobbleGoalCamera() {
        int wobbleGoalCameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.wobbleGoalCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, STACK_WEBCAM), wobbleGoalCameraMonitorViewId);
        this.wobbleGoalPipeline = new WobbleGoalPipeline();
        wobbleGoalCamera.setPipeline(wobbleGoalPipeline);
        wobbleGoalCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                wobbleGoalCamera.startStreaming(WEBCAM_WIDTH, WEBCAM_HEIGHT, WEBCAM_ROTATION);
            }
        });
        wobbleGoalCameraInitialized = true;
    }

    // Close the Wobble Goal Camera
    public void stopWobbleGoalCamera() {
        wobbleGoalCamera.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener()
        {
            @Override
            public void onClose() {}
        });
        wobbleGoalCameraInitialized = false;
    }

    // Check what StarterStack configuration is on the field
    public Constants.StarterStack checkStack() {
        if (stackCameraInitialized) {
            double area = stackPipeline.getStarterStack().getArea();
            if (area > MIN_STARTERSTACK_QUAD_AREA) {
                return Constants.StarterStack.QUAD;
            } else if (area > MIN_STARTERSTACK_SINGLE_AREA){
                return Constants.StarterStack.SINGLE;
            }
        }
        return Constants.StarterStack.NONE;
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

    // Get the Powershot Detection
    public PowershotDetection getPowershots() {
        return targetingCameraInitialized ? targetingPipeline.getPowershots() : INVALID_POWERSHOT_DETECTION;
    }

    // Get the number of frames the current camera has processed
    public int getFrameCount() {
        if (stackCameraInitialized) {
            return stackCamera.getFrameCount();
        } else if (targetingCameraInitialized) {
            return targetingCamera.getFrameCount();
        } else if (wobbleGoalCameraInitialized) {
            return wobbleGoalCamera.getFrameCount();
        } else {
            return 0;
        }
    }

    // Get Telemetry for the current active camera
    public String getTelemetry() {
        if (stackCameraInitialized) {
            return String.format(Locale.US, "Stack: %s", checkStack());
        } else if (targetingCameraInitialized) {
            return String.format(Locale.US, "PowerShots: (%.1f,%.1f) (%.1f,%.1f) (%.1f,%.1f)\n" +
                                                    "Red Goal:  Area: %.2f Center: (%.2f,%.2f)\n" +
                                                    "Blue Goal: Area: %.2f Center: (%.2f,%.2f)",
                    targetingPipeline.getPowershots().get(0).getCenter().x, targetingPipeline.getPowershots().get(0).getCenter().y,
                    targetingPipeline.getPowershots().get(1).getCenter().x, targetingPipeline.getPowershots().get(1).getCenter().y,
                    targetingPipeline.getPowershots().get(2).getCenter().x, targetingPipeline.getPowershots().get(2).getCenter().y,
                    targetingPipeline.getRed().getArea(), targetingPipeline.getRed().getCenter().x, targetingPipeline.getRed().getCenter().x,
                    targetingPipeline.getBlue().getArea(), targetingPipeline.getBlue().getCenter().x, targetingPipeline.getBlue().getCenter().x);
        }
        return ("No Camera Initialized");
    }
}