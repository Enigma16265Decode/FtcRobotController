package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "wheel tester swag")
public class WheelWireTesterSwag extends OpMode {
    String wheel1Name = "leftShooter", wheel2Name = "leftFront", wheel3Name = "rightRear", wheel4Name = "rightFront";
    DcMotor wheel1, wheel2, wheel3, wheel4;

    @Override
    public void init() {
        wheel1 = hardwareMap.get(DcMotor.class, wheel1Name);
        wheel2 = hardwareMap.get(DcMotor.class, wheel2Name);
        wheel3 = hardwareMap.get(DcMotor.class, wheel3Name);
        wheel4 = hardwareMap.get(DcMotor.class, wheel4Name);
    }

    @Override
    public void start() {
    }
    private void runWheels() {
        double power = 0.2;
        if(gamepad1.x) {
            wheel1.setPower(power);
        }
        else {
            wheel1.setPower(0);
        }

        if(gamepad1.y) {
            wheel2.setPower(power);
        }
        else {
            wheel2.setPower(0);
        }

        if(gamepad1.b) {
            wheel3.setPower(power);
        }
        else {
            wheel3.setPower(0);
        }

        if(gamepad1.a) {
            wheel4.setPower(power);
        }
        else {
            wheel4.setPower(0);
        }
    }

    private void runTelemetry() {
        telemetry.addData("X", wheel1Name);
        telemetry.addData("Y", wheel2Name);
        telemetry.addData("B", wheel3Name);
        telemetry.addData("A", wheel4Name);
    }

    @Override
    public void loop() {
        runTelemetry();
        runWheels();
    }
}
