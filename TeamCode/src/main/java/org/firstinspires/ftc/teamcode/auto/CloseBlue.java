package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.enums.Alliances;

@Autonomous(name = "Allen C. Bungalow (Alliance Close Blue)")
public class CloseBlue extends DautoClose {
    public CloseBlue() {
        super(Alliances.BLUE);
    }
}
