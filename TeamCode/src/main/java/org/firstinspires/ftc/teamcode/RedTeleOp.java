package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.enums.Alliances;

@TeleOp(name = "! RedTeleOp")
public class RedTeleOp extends MainTeleOp{
    public RedTeleOp() {
        super(Alliances.RED);
    }
}
