package org.firstinspires.ftc.teamcode.teleopClasses;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
    private final Gamepad gamepad1;
    private final HardwareMap hardwareMap;
    private Turret turret;
    public static double sP = 0.018, sI = 0.35 /*0.72 */, sD = 0; //we will almost certainly not change d, change p after finding good i value
    public static int targetSpeed = 1200;
    public double gateClosed = 0.4;
    public double gateOpen = 0.2;
    boolean shootToggle = false;
    PIDController shooterController;

    private DcMotorEx primaryShooter;
    private DcMotor secondaryShooter;
    private Servo hoodLeft;
    private Servo gate;


    public Shooter(@NonNull HardwareMap hardwareMap, Gamepad gamepad1, Turret turret) {
        shooterController = new PIDController(sP, sI, sD);

        primaryShooter = hardwareMap.get(DcMotorEx.class, "leftShooter"); //change depending on side
        secondaryShooter = hardwareMap.get(DcMotor.class, "rightShooter");
        hoodLeft = hardwareMap.get(Servo.class, "leftHood");
        gate = hardwareMap.get(Servo.class, "gate");

        hoodLeft.setDirection(Servo.Direction.REVERSE);
        primaryShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        this.gamepad1 = gamepad1;
        this.hardwareMap = hardwareMap;
        this.turret = turret;
    }

    public void setShooterPower(double value) {
        primaryShooter.setPower(value);
        secondaryShooter.setPower(value);
    }

    public void shooterController() {
        double currentVelocity = primaryShooter.getVelocity();

        if(gamepad1.b && gamepad1.bWasPressed()) {
            boolean stateBeforeToggle = shootToggle;
            if(stateBeforeToggle) {
                shootToggle = false;
            }
            else {
                shootToggle = true;
            }
        }
        if(gamepad1.right_bumper || shootToggle) {
            shooterController.setPID(sP, sI, sD);
            double shooterPid = shooterController.calculate(currentVelocity, targetSpeed);

            setShooterPower(shooterPid);
            gate.setPosition(gateOpen);
        }
        else {
            setShooterPower(0);
            gate.setPosition(gateClosed);
        }
    }
    public void turretController() {
        turret.setTargetBasedOnHeadingToGoal();
        turret.moveTurret();
    }

    public void gateController() {
        if(gamepad1.x && gamepad1.xWasPressed()) {
            toggleGate();
        }
    }


    private void setHoodPos(double value) {
        hoodLeft.setPosition(value);
    }

    public void hoodControl() {
        double amountToMove = 0.05;

        if(gamepad1.dpad_right && gamepad1.dpadRightWasPressed()) {
            double toSet = (hoodLeft.getPosition() - amountToMove);
            if (toSet < 0.15) {
                setHoodPos(0.15);
            }
            else {
                setHoodPos(toSet);
            }
        }
        if(gamepad1.dpad_left && gamepad1.dpadLeftWasPressed()) {
            double toSet = (hoodLeft.getPosition() + amountToMove);

            if (toSet > 1) {
                setHoodPos(1);
            }
            else {
                setHoodPos(toSet);
            }
        }
    }

    public void toggleGate() {
        boolean hasToggled = false;

        if(gate.getPosition() == gateOpen && hasToggled == false) {
            gate.setPosition(gateClosed);
            hasToggled = true;
        }
        if(gate.getPosition() == gateClosed && hasToggled == false) {
            gate.setPosition(gateOpen);
            hasToggled = false;
        }
        else {
            gate.setPosition(gateClosed);
        }
    }

    public double getShooterVelocity() {
        return primaryShooter.getVelocity();
    }

    public double getShooterPower() {
        return primaryShooter.getPower();
    }

    public double getHoodPos() {
        return hoodLeft.getPosition();
    }
}
