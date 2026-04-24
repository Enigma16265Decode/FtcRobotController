package org.firstinspires.ftc.teamcode.teleop.variants;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.enums.Alliances;
import org.firstinspires.ftc.teamcode.teleop.MainTeleOp;

@TeleOp(name = "! BlueTeleOp")
public class BlueTeleOp extends MainTeleOp {
    public BlueTeleOp() {
        super(Alliances.BLUE);
    }
}
