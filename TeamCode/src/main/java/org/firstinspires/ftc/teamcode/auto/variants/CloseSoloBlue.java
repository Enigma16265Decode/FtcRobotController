package org.firstinspires.ftc.teamcode.auto.variants;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.DautoCloseSolo;
import org.firstinspires.ftc.teamcode.enums.Alliances;

@Autonomous(name = "Sally C. Brown (Solo Close Blue)")
public class CloseSoloBlue extends DautoCloseSolo {
    public CloseSoloBlue() {
        super(Alliances.BLUE);
    }
}
