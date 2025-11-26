package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name= "!Shooter Tester")
public class ShooterTest extends LinearOpMode {
    DcMotorEx pShooter;
    DcMotor sShooter;
    Servo hoodLeft; //will be "primary"
    Servo hoodRight;
    PIDController controller;
    double pid;
    public double p = 0, i = 0, d = 0;
    double targetShooterVelocity = 10/13.8;


    private void setHoodPos(double posToSet) {
        hoodLeft.setPosition(posToSet);
        hoodRight.setPosition(posToSet);
    }

    private void setShooterPower(double powerToSet) {
        pShooter.setPower(powerToSet);
        sShooter.setPower(powerToSet);
    }

    private void hoodControl() {
        double amountToMove = 0.1;

        if(gamepad1.left_bumper && gamepad1.leftBumperWasPressed()) {
            double toSet = (hoodLeft.getPosition() - amountToMove);
            if (toSet < 0.2) {
                setHoodPos(0.2);
            }
            else {
                setHoodPos(toSet);
            }
        }
        if(gamepad1.right_bumper && gamepad1.rightBumperWasPressed()) {
            double toSet = (hoodLeft.getPosition() + amountToMove);

            if (toSet > 0.9) {
                setHoodPos(0.9);
            }
            else {
                setHoodPos(toSet);
            }
        }
    }

    private void shooterControl() {
        double amountToMove = 0.1;

        if(gamepad1.x && gamepad1.xWasPressed()) {
            setShooterPower(pShooter.getPower() - amountToMove);
        }
        if(gamepad1.b && gamepad1.bWasPressed()) {
            setShooterPower(pShooter.getPower() + amountToMove);
        }
        if(gamepad1.y && gamepad1.yWasPressed()) {
            setShooterPower(1);
        }
        if(gamepad1.a && gamepad1.aWasPressed()) {
            setShooterPower(0);
        }
    }

    private void shooterController() {
        controller.setPID(p, i, d);
        double currentVelocity = pShooter.getVelocity();
        pid = controller.calculate(currentVelocity, targetShooterVelocity);

        pShooter.setPower(pid);
        sShooter.setPower(pid);
    }

    private void telemetryMain() {
        telemetry.addData("Shooter power: ", pShooter.getPower());
        telemetry.addData("Hood Pos: ", hoodLeft.getPosition());

        telemetry.update();
    }

    private void initPID() {
        controller = new PIDController(p, i, d);
        //telemetry
    }

    private void initializeHardware() {
        pShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        sShooter = hardwareMap.get(DcMotor.class, "rightShooter");
        hoodLeft = hardwareMap.get(Servo.class, "leftHood");
        hoodRight = hardwareMap.get(Servo.class, "rightHood");

        pShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        hoodRight.setDirection(Servo.Direction.REVERSE);
    }

    private void mainLoop() {
        hoodControl();
        shooterControl();

        telemetryMain();
    }

    @Override
    public void runOpMode() {
        initializeHardware();

        waitForStart();

        while(opModeIsActive()) {
            mainLoop();
        }
    }
}
