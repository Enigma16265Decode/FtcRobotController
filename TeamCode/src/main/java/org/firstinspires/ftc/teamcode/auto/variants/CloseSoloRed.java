package org.firstinspires.ftc.teamcode.auto.variants;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.DautoCloseSolo;
import org.firstinspires.ftc.teamcode.enums.Alliances;

@Autonomous(name = "Sharon C. Robinson (Solo Close Red)")
public class CloseSoloRed extends DautoCloseSolo {
    public CloseSoloRed() {
        super(Alliances.RED);
    }
}
