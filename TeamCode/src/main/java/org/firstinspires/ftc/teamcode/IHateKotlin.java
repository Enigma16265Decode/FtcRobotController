package org.firstinspires.ftc.teamcode;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "kotlin sucks")
public class IHateKotlin extends LinearOpMode {
    static TelemetryManager telemetryM;
    int crazynumber = 0;

    @Override
    public void runOpMode() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            telemetryM.addData("crazy number :", crazynumber);
            crazynumber++;
            telemetryM.update();
        }
    }
}
