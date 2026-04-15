package org.firstinspires.ftc.teamcode.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "! servo tuner basic")
public class ServoTunerBasic extends OpMode {
    private Servo turretRight, turretLeft;

    @Override
    public void init() {
        turretLeft = hardwareMap.get(Servo.class, "turretLeft");
        turretRight = hardwareMap.get(Servo.class, "turretRight");
        turretRight.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        if(gamepad1.aWasPressed()) {
            turretRight.setPosition(0);
            turretLeft.setPosition(0);
        }
        if(gamepad1.bWasPressed()) {
            turretRight.setPosition(0.5);
            turretLeft.setPosition(0.5);
        }
        if(gamepad1.xWasPressed()) {
            turretRight.setPosition(0.75);
            turretLeft.setPosition(0.75);
        }
        if(gamepad1.yWasPressed()) {
            turretRight.setPosition(1);
            turretLeft.setPosition(1);
        }
        telemetry.addData("turretPos", turretRight.getPosition());
        telemetry.update();
    }
}
