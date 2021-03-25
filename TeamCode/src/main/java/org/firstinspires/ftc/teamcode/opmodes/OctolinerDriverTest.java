package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Octoliner;

@TeleOp(name = "Octoliner Test", group = "Development")
public class OctolinerDriverTest extends OpMode {

    Octoliner octoliner;

    @Override
    public void init() {
        this.octoliner = this.hardwareMap.get(Octoliner.class, "octoliner");
    }

    @Override
    public void loop() {
        this.telemetry.addData("Analog", octoliner.analogReadString());
        this.telemetry.addData("Digital", octoliner.digitalReadString());
        this.telemetry.addData("Ring Count", octolinerToRingCount(octoliner.digitalSum()));
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