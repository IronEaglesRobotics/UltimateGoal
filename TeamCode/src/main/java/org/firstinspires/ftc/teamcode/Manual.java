package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static android.os.SystemClock.sleep;

/*Terms:
* - ms ... Milisecond(s).
* - Telemetry ... Recording and or transmission of the readings of an instrument. Basically: enviornment sensing.
* - TFOD ... TensorFlowObjectDetector. Invokes TensorFlow's object detection API, assumably. Read more: https://github.com/tensorflow/models/tree/master/research/object_detection.
*/

// manual driver control
@TeleOp(name = "Manual")
public class Manual extends OpMode {
    public int msStuckDetectInit = 15000;

    private Robot robot;
    private boolean clawPressed;
    private boolean pusherPressed;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
//        robot.setTfodZoom(3);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    double startTime = 0;
    double endTime = 0;

    @Override
    public void loop() {
        // driver 1
        if (gamepad1.right_bumper)
        {
            robot.drive.setInput(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        }
        else
        {
            robot.drive.setInput(gamepad1.left_stick_x*0.7, -gamepad1.left_stick_y*0.7, gamepad1.right_stick_x*0.7);
        }


//Move the reverse intake thing to l_bumber


//      // driver 2
        if (gamepad2.right_stick_y > 0)
        {
            robot.arm.setArm(-0.25);
        } else if (gamepad2.right_stick_y < 0)
        {
            robot.arm.setArm(0.25);
        }
        else
        {
            robot.arm.setArm(0);
        }

        if (gamepad2.b && !clawPressed) {
            robot.arm.setClaw(!robot.arm.getClaw());
        }
        clawPressed = gamepad2.b;


        //Legacy
        // if (gamepad2.right_bumper) {
        //     robot.arm.setArm(0.25);
        // } else if (gamepad2.left_bumper) {
        //     robot.arm.setArm(-0.25);
        // } else {
        //     robot.arm.setArm(0);
        // }
        // if (gamepad2.b && !clawPressed) {
        //     robot.arm.setClaw(!robot.arm.getClaw());
        // }
        // clawPressed = gamepad2.b;

        if (gamepad2.left_trigger > 0)
        {
            robot.intake.setIntake(1);
        }
        else if(gamepad2.left_bumper)
        {
            robot.intake.setIntake(-1);
        }

        // if (gamepad2.x) {
        //     robot.intake.setIntake(-gamepad2.left_trigger);
        // } else {
        //     robot.intake.setIntake(gamepad2.left_trigger);
        // }

        if (gamepad2.y) {
            robot.shooter.setShooter(-gamepad2.right_trigger*0.7);
        } else {
            robot.shooter.setShooter(gamepad2.right_trigger*0.7);
        }

        //Open and close the pusher.
        if (gamepad2.a)
        {
            startTime = getRuntime();
            endTime = startTime + 10000;
        }
        if(!(endTime > getRuntime()))
        {
            robot.shooter.setPusher(false);
        }
        else
        {
            robot.shooter.setPusher(true);
        }

        telemetry.addData("Shooter pos: ", robot.shooter.getTelemetry());

        /*
        //Some legacy...

        if (gamepad2.a && !pusherPressed) {
            robot.shooter.setPusher(!robot.shooter.getPusher());
        }
        pusherPressed = gamepad2.a;
        */

        // show telemetry
        telemetry.addData("Status", robot.getTelemetry());
        telemetry.update();
    }
}
