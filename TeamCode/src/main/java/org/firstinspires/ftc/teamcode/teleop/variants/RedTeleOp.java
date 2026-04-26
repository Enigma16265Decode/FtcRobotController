package org.firstinspires.ftc.teamcode.teleop.variants;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.DautoClose;
import org.firstinspires.ftc.teamcode.auto.DautoCloseSolo;
import org.firstinspires.ftc.teamcode.enums.Alliances;
import org.firstinspires.ftc.teamcode.teleop.MainTeleOp;

@TeleOp(name = "! RedTeleOp")
public class RedTeleOp extends MainTeleOp {
    public RedTeleOp() {
        super(Alliances.RED, DautoCloseSolo.parkPose);
    }
}
