package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/*Terms:
* - OpenCvCameraFactory ... From FTC's EasyOpenCV. Makes it easier to get camera IDs, assumably.
* - OpenCvCamera ... Another camera extension from FTC robotics.
*/

//Executes OpenCV code (hence: "TeleOp").
@TeleOp
public class CVSandbox extends LinearOpMode
{
    //private Robot robot;

    OpenCvCamera webcam;
    CVPipeline pipeline;

    @Override
    public void runOpMode()
    {
        //robot = new Robot(hardwareMap);


        //Find the camera and add it as a variable.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        this.pipeline = new CVPipeline();
        webcam.setPipeline(pipeline);

        //Create asynchronous camera stream in the app using EasyOpenCV.
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

        while(!opModeIsActive() && webcam.getFrameCount() == 0) {
            this.sleep(1);
        }

        waitForStart();

        Detection red = null;
        Detection blue = null;

        //Start LinearOpMode.
        while (opModeIsActive())
        {
            red = pipeline.getRed();
            blue = pipeline.getBlue();

            double x = 0;
            double y = 0;
            double z = 0;

            if (gamepad1.right_bumper) {
                // Aim for goal

                double zMaxSpeed = 0.7;
                double zErr = Math.abs(red.getCenter().x);
                double zSpeed = (zErr / 50) * zMaxSpeed;
                if (zErr <= 1) {
                    z = 0;
                } else if (zErr > 1) {
                    z = Math.copySign(zSpeed, red.getCenter().x);
                }

//                double yMaxSpeed = 0.7;
//                double yErr = Math.abs(5-red.getArea());
//                double speed = (yErr / 5) * yMaxSpeed;
//                if (yErr <= 0.1) {
//                    y = 0;
//                } else if (yErr > 0.1) {
//                    y = Math.copySign(speed, 5-red.getArea());
//                }
                telemetry.addData("FPS", String.format("%.1f", webcam.getFps()));
                telemetry.addData("Red", String.format("Area: %.1f, Center: (%.1f, %.1f)", red.getArea(), red.getCenter().x, red.getCenter().y));
                telemetry.addData("Blue", String.format("Area: %.1f, Center: (%.1f, %.1f)", blue.getArea(), blue.getCenter().x, blue.getCenter().y));
                telemetry.update();
            }
            if (gamepad1.left_bumper) {
                telemetry.addData("FPS", String.format("%.1f", webcam.getFps()));
                telemetry.addData("Red", String.format("Area: %.1f, Center: (%.1f, %.1f)", red.getArea(), red.getCenter().x, red.getCenter().y));
                telemetry.addData("Blue", String.format("Area: %.1f, Center: (%.1f, %.1f)", blue.getArea(), blue.getCenter().x, blue.getCenter().y));
                // Aim for powershot
                PowerShotDetection powershots = pipeline.getPowerShots();
                int count = powershots.getCount();
                telemetry.addData("PS Count", powershots.getCount());
                if (count > 0) {
//                    double zMaxSpeed = 0.7;
//                    double zErr = Math.abs(powershots.get(0).getCenter().x);
//                    double zSpeed = (zErr / 50) * zMaxSpeed;
//                    if (zErr <= 1) {
//                        z = 0;
//                    } else if (zErr > 1) {
//                        z = Math.copySign(zSpeed, red.getCenter().x);
//                    }

                    telemetry.addData("First PS Center", powershots.get(0).getCenter());
                }
                telemetry.update();
            }

            //robot.drive.setInput(x, y, z);

            /*
             * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
             * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
             * anyway). Of course in a real OpMode you will likely not want to do this.
             */
            sleep(100);
        }
    }
}