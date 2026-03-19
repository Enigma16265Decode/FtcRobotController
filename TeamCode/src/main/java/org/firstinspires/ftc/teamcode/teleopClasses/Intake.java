package org.firstinspires.ftc.teamcode.teleopClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake { //This also does transfer
    private DcMotor frontIntake, backIntake;
    private Gamepad gamepad1;
    private HardwareMap hardwareMap;
    private Shooter shooter;

    public Intake(HardwareMap hardwareMap, Gamepad gamepad1, Shooter shooter) {
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        backIntake = hardwareMap.get(DcMotor.class, "backIntake");

        this.gamepad1 = gamepad1;
        this.hardwareMap = hardwareMap;
        this.shooter = shooter;
    }

    public void setIntakePower(double toSet) {
        frontIntake.setPower(toSet);
        backIntake.setPower(toSet/2);
    }

    /** Runs the intake based on gamepad input **/
    public void intakeController() {
        if(gamepad1.right_trigger > 0.4) {
            frontIntake.setPower(1);
            backIntake.setPower(0.6);
        }
        else {
            if(gamepad1.left_trigger > 0.4) {
                frontIntake.setPower(1);
                backIntake.setPower(-0.6);
            }
            else {
                frontIntake.setPower(0);
                backIntake.setPower(0);
            }
        }
        if(gamepad1.x) {
            if(shooter.isShooterAtSpeed()) {
                frontIntake.setPower(1);
                backIntake.setPower(1);
            }
        }
    }
}
