package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

//Where the OpenCV code is executed (for now)?
@TeleOp
public class CVSandbox extends LinearOpMode
{
    private Robot robot;

    OpenCvCamera webcam;
    CVPipeline pipeline;

    @Override
    public void runOpMode()
    {
        robot = new Robot(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        this.pipeline = new CVPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            Detection red = pipeline.getRed();
            Detection blue = pipeline.getBlue();

            double x = 0;
            double y = 0;
            double z = 0;

//            if (red.getCenter().x < -window) {
//                z = -0.1;
//            } else if (red.getCenter().x > window) {
//                z = 0.1;
//            } else if (red.getCenter() == Detection.INVALID_POINT || Math.abs(red.getCenter().x) < window) {
//                z = 0;
//            }
            double xMaxSpeed = 0.7;
            double xErr = Math.abs(red.getCenter().x);
            double xSpeed = (xErr / 50) * xMaxSpeed;
            if (xErr <= 1) {
                z = 0;
            } else if (xErr > 1) {
                z = Math.copySign(xSpeed, red.getCenter().x);
            }

            double yMaxSpeed = 0.7;
            double yErr = Math.abs(5-red.getArea());
            double speed = (yErr / 5) * yMaxSpeed;
            if (yErr <= 0.1) {
                y = 0;
            } else if (yErr > 0.1) {
                y = Math.copySign(speed, 5-red.getArea());
            }

//            if (red.getArea() < 4.7) {
//                y = 0.2;
//            } else if (red.getArea() > 5.3) {
//                y = -0.2;
//            } else if (red.getCenter() == Detection.INVALID_POINT || Math.abs(5.0-red.getArea()) < 0.3) {
//                y = 0;
//            }
//            if (red.getArea() < 2) {
//                y = 1;
//            } else if (red.getArea() > 8) {
//                y = -1;
//            } else if (red.getArea() < 5.1) {
//                z = red.getArea()/2.9;
//            } else if (red.getArea() > 5.1) {
//                z = -red.getArea()/2.9;
//            } else if (Math.abs(5-red.getArea()) <= 0.1) {
//                z = 0;
//            }

            robot.drive.setInput(x, y, z);

            telemetry.addData("FPS", String.format("%.1f", webcam.getFps()));
            telemetry.addData("Red", String.format("Area: %.1f, Center: (%.1f, %.1f)", red.getArea(), red.getCenter().x, red.getCenter().y));
            telemetry.addData("Blue", String.format("Area: %.1f, Center: (%.1f, %.1f)", blue.getArea(), blue.getCenter().x, blue.getCenter().y));
            telemetry.update();

            /*
             * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
             * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
             * anyway). Of course in a real OpMode you will likely not want to do this.
             */
            sleep(100);
        }
    }
}