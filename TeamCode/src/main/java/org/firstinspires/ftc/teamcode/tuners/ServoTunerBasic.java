package org.firstinspires.ftc.teamcode.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "! servo tuner basic")
public class ServoTunerBasic extends OpMode {
    private Servo turretRight, turretLeft, hood;

    @Override
    public void init() {
        turretLeft = hardwareMap.get(Servo.class, "turretLeft");
        turretRight = hardwareMap.get(Servo.class, "turretRight");
        turretRight.setDirection(Servo.Direction.REVERSE);

        hood = hardwareMap.get(Servo.class, "hood");
    }

    public void setTurretPos(double pos) {
        turretRight.setPosition(pos);
        turretLeft.setPosition(pos);
        //hood.setPosition(pos);
    }

    @Override
    public void loop() {
        if(gamepad1.aWasPressed()) {
            setTurretPos(0);
        }
        if(gamepad1.bWasPressed()) {
            setTurretPos(0.5);
        }
        if(gamepad1.xWasPressed()) {
            setTurretPos(0.75);
        }
        if(gamepad1.yWasPressed()) {
            setTurretPos(1);
        }
        if(gamepad1.rightBumperWasPressed()) {
            setTurretPos(turretLeft.getPosition() + 0.01);
        }
        if(gamepad1.leftBumperWasPressed()) {
            setTurretPos(turretLeft.getPosition() - 0.01);
        }
        telemetry.addData("turretPos", turretRight.getPosition());
        telemetry.update();
    }
}
