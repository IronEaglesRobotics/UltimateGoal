package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class StarterStackDetector {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY ="AUmjH6X/////AAABmeSd/rs+aU4giLmf5DG5vUaAfHFLv0/vAnAFxt5vM6cbn1/nI2sdkRSEf6HZLA/is/+VQY5/i6u5fbJ4TugEN8HOxRwvUvkrAeIpgnMYEe3jdD+dPxhE88dB58mlPfVwIPJc2KF4RE7weuRBoZ8KlrEKbNNu20ommdG7S/HXP9Kv/xocj82rgj+iPEaitftALZ6QaGBdfSl3nzVMK8/KgQJNlSbGic/Wf3VI8zcYmMyDslQPK45hZKlHW6ezxdGgJ7VJCax+Of8u/LEwfzqDqBsuS4/moNBJ1mF6reBKe1hIE2ffVTSvKa2t95g7ht3Z4M6yQdsI0ZaJ6AGnl1wTlm8Saoal4zTbm/VCsmZI081h";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public StarterStackDetector(HardwareMap hardwareMap) {
        this.initVuforia(hardwareMap);
        this.initTfod(hardwareMap);
    }

    public StarterStack checkStack() {
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getRecognitions();
            if (updatedRecognitions != null && updatedRecognitions.size() == 1) {
                Recognition recognition = updatedRecognitions.get(0);
                switch(recognition.getLabel()) {
                    case LABEL_SECOND_ELEMENT:
                        return StarterStack.SINGLE;
                    case LABEL_FIRST_ELEMENT:
                        return StarterStack.QUAD;
                    default:
                        return StarterStack.NONE;
                }
            }
        }
        return StarterStack.NONE;
    }

    public enum StarterStack {
        NONE, SINGLE, QUAD
    }

    private void initVuforia(HardwareMap hardwareMap) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        //parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        if (tfod != null) {
            tfod.activate();
        }
    }
}