package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.bosch.NaiveAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode;

//For ADB Connection: `adb connect 192.168.43.1:5555`

/*Terms:
* - IMU ... Inertial Measurement Unit. Measures an object's change in motion, among other things.
* - BNO055IMU ... A type of IMU sensor from Bosch hardware.
*/

//Robot init and setup class.
public class Robot {
    public MecanumDrive drive;
    public Arm arm;
    public Intake intake;
    public Shooter shooter;
//    private StarterStackDetector stackDetector;

    // The IMU sensor object
    private BNO055IMU imu;

    // State used for updating telemetry
    private Orientation angles;
    private Acceleration gravity;

    //Give the wheels, intake, shooter, arm, and starter stack detector their hardware. Also set up IMU.
    public Robot(HardwareMap hardwareMap) {
        drive = new MecanumDrive(hardwareMap);
        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
//        stackDetector = new StarterStackDetector(hardwareMap);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        this.imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = false;
        this.imu.initialize(parameters);
    }

    private float baseGyroHeading;

    public void resetGyroHeading() {
        baseGyroHeading = getGyroHeading180();
    }

    public float getGyroHeading360() {
        float euler =  getGyroHeading180();
        return MathHelpers.piTo2Pi(euler);
    }

    public float getGyroHeading180() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - baseGyroHeading;
    }

//    public void setTfodZoom(int zoom) {
//        this.stackDetector.setZoom(zoom);
//    }
//
//    public StarterStackDetector.StarterStack checkStack() {
//        return stackDetector.checkStack();
//    }
//
//    public void shutdownVuforia() {
//        stackDetector.shutdown();
//    }

    //Get telemetry for all relevant hardware.
    public Telemetry getTelemetry() {
        Telemetry t = new Robot.Telemetry();
        t.setDriveStatus(drive.getTelemetry());
        t.setArmStatus(arm.getTelemetry());
        t.setIntakeStatus(intake.getTelemetry());
        t.setShooterStatus(shooter.getTelemetry());
//        t.setRingStatus("Ring: "+stackDetector.checkStack());
        return t;
    }

    //Setters and getters for drive, arm, and ring status.
    public class Telemetry {
        private String driveStatus;
        private String armStatus;
        private String intakeStatus;
        private String shooterStatus;
        private String ringStatus;

        public void setDriveStatus(String driveStatus) {
            this.driveStatus = driveStatus;
        }

        public void setArmStatus(String armStatus) {
            this.armStatus = armStatus;
        }

        public void setIntakeStatus(String armStatus) {
            this.intakeStatus = armStatus;
        }

        public void setShooterStatus(String armStatus) {
            this.shooterStatus = armStatus;
        }

        public void setRingStatus(String ringStatus) {
            this.ringStatus = ringStatus;
        }

        @Override
        public String toString() {
            return String.format("\n%s\n%s\n%s\n%s\n%s", driveStatus, armStatus, intakeStatus, shooterStatus, ringStatus);
//            return String.format("\n%s\n%s\n%s\n%s", driveStatus, intakeStatus, shooterStatus, ringStatus);
        }
    }
}
