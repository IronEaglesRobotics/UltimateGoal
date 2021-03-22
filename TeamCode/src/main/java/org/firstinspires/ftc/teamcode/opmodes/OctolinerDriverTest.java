package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.hardware.Octoliner;

@TeleOp(name = "Octoliner Test", group = "Development")
public class OctolinerDriverTest extends OpMode {

    Octoliner octoliner;

    @Override
    public void init() {
        this.octoliner = this.hardwareMap.get(Octoliner.class, "octoliner");
    }

    private int lastUpdated = 0;

    @Override
    public void loop() {
        this.telemetry.addData("Analog", octoliner.analogReadAllString());
        this.telemetry.addData("Digital", octoliner.digitalReadAllString());
        this.telemetry.addData("Ring Count", octolinerToRingCount(octoliner.digitalCount()));
        this.telemetry.update();
    }

    private String octolinerToRingCount(int count) {
        if (count >= 7) {
            return "NO RINGS";
        } else if (count >= 5) {
            return "ONE";
        } else if (count >= 2) {
            return "TWO";
        } else {
            return "THREE";
        }
    }
}
