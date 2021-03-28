package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.enums.Alliance;
import org.firstinspires.ftc.teamcode.opmode.util.Tele;

@Disabled
@TeleOp(name = "Blue TeleOp", group = "Competition")
public class BlueTele extends Tele {
    @Override
    public void setAlliance() {
        this.alliance = Alliance.RED;
    }
}
