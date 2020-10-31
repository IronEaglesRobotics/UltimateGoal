/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Mecanum Test", group="Testing")
public class TestMecanum extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor rightFrontDrive = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftBackPower;
            double leftFrontPower;
            double rightBackPower;
            double rightFrontPower;

            // Test motor direction based on gamepad stick
            // double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            // double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            // double rightX = gamepad1.right_stick_x;
            // leftBackPower = r * Math.sin(robotAngle) + rightX;
            // leftFrontPower = r * Math.cos(robotAngle) + rightX;
            // rightBackPower = r * Math.cos(robotAngle) - rightX;
            // rightFrontPower = r * Math.sin(robotAngle) - rightX;

            double drive = -gamepad1.left_stick_y;
            leftBackPower    = Range.clip(drive, -1.0, 1.0) ;
            leftFrontPower   = Range.clip(drive, -1.0, 1.0) ;
            rightBackPower    = Range.clip(drive, -1.0, 1.0) ;
            rightFrontPower   = Range.clip(drive, -1.0, 1.0) ;

            // Send calculated power to wheels
            leftBackDrive.setPower(leftBackPower);
            leftFrontDrive.setPower(leftFrontPower);
            rightBackDrive.setPower(rightBackPower);
            rightFrontDrive.setPower(rightFrontPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left back (%.2f), left front (%.2f)", leftBackPower, leftFrontPower);
            telemetry.addData("Motors", "right back (%.2f), right front (%.2f)", rightBackPower, rightFrontPower);
            telemetry.update();
        }
    }
}
