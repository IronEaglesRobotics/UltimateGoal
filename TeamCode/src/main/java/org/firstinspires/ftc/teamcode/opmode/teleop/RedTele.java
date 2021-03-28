package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.enums.Alliance;
import org.firstinspires.ftc.teamcode.opmode.util.Tele;

@TeleOp(name = "Red TeleOp", group = "Competition")
public class RedTele extends Tele {
    @Override
    public void setAlliance() {
        this.alliance = Alliance.RED;
    }
}
