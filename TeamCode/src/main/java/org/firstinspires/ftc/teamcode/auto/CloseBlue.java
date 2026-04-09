package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.enums.Alliances;

@Autonomous(name = "Christopher Brown (Close Blue)")
public class CloseBlue extends DautoClose {
    public CloseBlue() {
        super(Alliances.BLUE);
    }
}
