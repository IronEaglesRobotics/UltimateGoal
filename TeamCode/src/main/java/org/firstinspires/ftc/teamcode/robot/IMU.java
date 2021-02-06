package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.MathHelpers;

import static org.firstinspires.ftc.teamcode.Constants.IMU_SENSOR;

// Class for the IMU Sensor on the Control Hub
public class IMU {
    private BNO055IMU imu;
    private float baseGyroHeading;

    // Constructor
    public IMU(HardwareMap hardwareMap) {
        this.imu = hardwareMap.get(BNO055IMU.class, IMU_SENSOR);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = false;
        this.imu.initialize(parameters);
    }

    // Reset the Gyro heading
    public void resetGyroHeading() {
        baseGyroHeading = getGyroHeading180();
    }

    // Get the heading out of 360 degrees (0 is default, counterclockwise is increasing from 0 to 360, clockwise is decreasing from 359 to 0)
    public float getGyroHeading360() {
        float euler =  getGyroHeading180();
        return MathHelpers.piTo2Pi(euler);
    }

    // Get the heading out of 180 degrees (0 is default, counterclockwise is increasing from 0 to 180, clockwise is decreasing from to 0 to -180)
    public float getGyroHeading180() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - baseGyroHeading;
    }
}
