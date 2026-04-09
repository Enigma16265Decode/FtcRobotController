package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.enums.Alliances;

@Autonomous(name = "Cindy Robinson (Close Red)")
public class CloseRed extends DautoClose {
    public CloseRed() {
        super(Alliances.RED);
    }
}
