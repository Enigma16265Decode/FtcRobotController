package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.enums.Alliances;

@TeleOp(name = "! BlueTeleOp")
public class BlueTeleOp extends MainTeleOp{
    public BlueTeleOp() {
        super(Alliances.BLUE);
    }
}
