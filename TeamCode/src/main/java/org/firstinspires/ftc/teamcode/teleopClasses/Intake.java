package org.firstinspires.ftc.teamcode.teleopClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake { //This also does transfer
    private DcMotor intake;
    private Gamepad gamepad1;
    private HardwareMap hardwareMap;
    private Shooter shooter;

    public Intake(HardwareMap hardwareMap, Gamepad gamepad1, Shooter shooter) {
        intake = hardwareMap.get(DcMotor.class, "intake");

        this.gamepad1 = gamepad1;
        this.hardwareMap = hardwareMap;
        this.shooter = shooter;
    }

    public void setIntakePower(double toSet) {
        intake.setPower(toSet);
    }

    public void intakeController() {
        if(gamepad1.right_trigger > 0.4) {
            intake.setPower(1);
        }
        else {
            if(gamepad1.left_trigger > 0.4) {
                intake.setPower(-1);
            }
            else {
                intake.setPower(0);
            }
        }
        if(gamepad1.x) {
            if(shooter.isShooterAtSpeed()) {
                intake.setPower(1);
            }
        }
        /*
        if(!gamepad1.a && gamepad1.right_trigger > 0.4) {
            setShooterPower(-0.4);
        }

         */
    }

    public double getIntakePower() {
        return intake.getPower();
    }
}
