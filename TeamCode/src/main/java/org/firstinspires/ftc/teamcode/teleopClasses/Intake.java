package org.firstinspires.ftc.teamcode.teleopClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.enums.IntakeModes;

public class Intake { //This also does transfer
    private DcMotor frontIntake, backIntake;
    private Servo intakeLift;
    private Gamepad gamepad1;
    private HardwareMap hardwareMap;
    private IntakeModes intakeMode;

    public Intake(HardwareMap hardwareMap, Gamepad gamepad1) {
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        backIntake = hardwareMap.get(DcMotor.class, "backIntake");
        intakeLift = hardwareMap.get(Servo.class, "intakeLift");
        frontIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeLift.setDirection(Servo.Direction.REVERSE);

        frontIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.gamepad1 = gamepad1;
        this.hardwareMap = hardwareMap;
    }

    public void setIntakeMode(IntakeModes intakeMode) {
        this.intakeMode = intakeMode;
    }

    public void runIntake() {
        if(intakeMode == IntakeModes.INTAKE) {
            frontIntake.setPower(1);
            backIntake.setPower(1);
        }
        if (intakeMode == IntakeModes.OUTTAKE) {
            frontIntake.setPower(-0.8);
            backIntake.setPower(-0.5);
        }
        if(intakeMode == IntakeModes.SHOOT) {
            frontIntake.setPower(1);
            backIntake.setPower(0.5);
        }
        if(intakeMode == IntakeModes.IDLE) {
            frontIntake.setPower(0);
            backIntake.setPower(0);
        }
        intakeLift.setPosition(0.65);
    }

    /** Runs the intake based on gamepad input **/
    public void intakeController() {
        if(gamepad1.x) {
            intakeMode = IntakeModes.SHOOT;
        }
        else {
            if(gamepad1.right_trigger > 0.4) {
                setIntakeMode(IntakeModes.INTAKE);
            }
            else {
                if(gamepad1.left_trigger > 0.4) {
                    setIntakeMode(IntakeModes.OUTTAKE);
                }
                else {
                    setIntakeMode(IntakeModes.IDLE);
                }
            }
        }
        runIntake();
    }
}
