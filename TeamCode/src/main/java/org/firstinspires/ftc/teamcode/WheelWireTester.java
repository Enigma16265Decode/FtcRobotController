package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "wheel wire tester")
public class WheelWireTester extends OpMode {

    DcMotor leftRear, leftFront, rightRear, rightFront;

    @Override
    public void init() {
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
    }

    @Override
    public void start() {

    }
    private void runWheels() {
        double power = 0.2;
        if(gamepad1.x) {
            leftRear.setPower(power);
        }
        if(gamepad1.y) {
            leftFront.setPower(power);
        }
        if(gamepad1.b) {
            rightRear.setPower(power);
        }
        if(gamepad1.a) {
            rightFront.setPower(power);
        }
    }

    private void runTelemetry() {
        telemetry.addData("leftRear = X", "");
        telemetry.addData("leftFront = Y", "");
        telemetry.addData("rightRear = B", "");
        telemetry.addData("rightFront = A", "");
    }

    @Override
    public void loop() {
        runTelemetry();
        runWheels();
    }
}
