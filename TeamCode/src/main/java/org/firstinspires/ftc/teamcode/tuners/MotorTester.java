package org.firstinspires.ftc.teamcode.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "wheel tester swag", group = "tuners")
public class MotorTester extends OpMode {
    String motor1Name = "leftRear", motor2Name = "leftFront", motor3Name = "rightRear", motor4Name = "rightFront";
    DcMotor motor1, motor2, motor3, motor4;

    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotor.class, motor1Name);
        motor2 = hardwareMap.get(DcMotor.class, motor2Name);
        motor3 = hardwareMap.get(DcMotor.class, motor3Name);
        motor4 = hardwareMap.get(DcMotor.class, motor4Name);
    }

    @Override
    public void start() {
    }
    private void runWheels() {
        double power = 0.4;
        if(gamepad1.x) {
            motor1.setPower(power);
        }
        else {
            motor1.setPower(0);
        }

        if(gamepad1.y) {
            motor2.setPower(power);
        }
        else {
            motor2.setPower(0);
        }

        if(gamepad1.b) {
            motor3.setPower(power);
        }
        else {
            motor3.setPower(0);
        }

        if(gamepad1.a) {
            motor4.setPower(power);
        }
        else {
            motor4.setPower(0);
        }
    }

    private void runTelemetry() {
        telemetry.addData("X", motor1Name);
        telemetry.addData("Y", motor2Name);
        telemetry.addData("B", motor3Name);
        telemetry.addData("A", motor4Name);
    }

    @Override
    public void loop() {
        runTelemetry();
        runWheels();
    }
}
