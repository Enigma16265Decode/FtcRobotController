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
    public static double sP = 0.02, sI = 0/*.35 /*0.72 */, sD = 0; //sP was 0.018, and sI was 0.35
    public static int targetSpeed = 1100;
    public double gateClosed = 0.0;
    public double gateOpen = 0.9;
    private boolean shootToggle = false;
    private boolean gateToggle = false;
    private boolean shooterStopped = false;
    private PIDController shooterController;

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

        secondaryShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        hoodLeft.setDirection(Servo.Direction.REVERSE);

        this.gamepad1 = gamepad1;
        this.hardwareMap = hardwareMap;
        this.turret = turret;
    }

    public boolean isShooterAtSpeed() {
        if((getShooterVelocity() > 1150) && (getShooterVelocity() < 1220)) {
            return true;
        }
        else {
            return false;
        }
    }

    public void setShooterPower(double value) {
        primaryShooter.setPower(value);
        secondaryShooter.setPower(value);
    }

    public void initHood() {
        setHoodPos(0.5);
    }

    public void accelerateShooterPID() {
        if (!shooterStopped) {
            double currentVelocity = primaryShooter.getVelocity() * -1;
            double shooterPid = shooterController.calculate(currentVelocity, targetSpeed);

            setShooterPower(shooterPid);
        }
        else {
            setShooterPower(0);
        }
    }

    public void shooterController() {
        double currentVelocity = primaryShooter.getVelocity() * -1;

        if(gamepad1.b && gamepad1.bWasPressed()) {
            boolean stateBeforeToggle = gateToggle;
            if(stateBeforeToggle) {
                gateToggle = false;
            }
            else {
                gateToggle = true;
            }
        }

        if(gamepad1.right_bumper && gamepad1.rightBumperWasPressed()) {
            boolean stateBeforeToggle = shootToggle;
            if(stateBeforeToggle) {
                shootToggle = false;
            }
            else {
                shootToggle = true;
            }
        }
        if(shootToggle) {
            shooterController.setPID(sP, sI, sD);
            double shooterPid = shooterController.calculate(currentVelocity, targetSpeed);

            setShooterPower(shooterPid);
        }
        else {
            setShooterPower(0);
        }
        if(gateToggle) {
            gate.setPosition(gateOpen);
        }
        else {
            gate.setPosition(gateClosed);
        }

    }
    public void turretController(boolean isRed) {
        turret.setTargetBasedOnHeadingToGoal(isRed);
        turret.moveTurret();
        turret.setOffset();
    }

    public void gateController() {
        if(gamepad1.x && gamepad1.xWasPressed()) {
            toggleGate();
        }
    }

    public void setShooterStopped(boolean stop) {
        shooterStopped = stop;
    }


    private void setHoodPos(double value) {
        hoodLeft.setPosition(value);
    }

    public void hoodControl() {
        final double lowestValue = 0.2;
        final double highestValue = 1.0;
        final double amountToMove = 0.05;

        if(gamepad1.dpad_right && gamepad1.dpadRightWasPressed()) {
            double toSet = (hoodLeft.getPosition() - amountToMove);
            if (toSet < lowestValue) {
                setHoodPos(lowestValue);
            }
            else {
                setHoodPos(toSet);
            }
        }
        if(gamepad1.dpad_left && gamepad1.dpadLeftWasPressed()) {
            double toSet = (hoodLeft.getPosition() + amountToMove);

            if (toSet > highestValue) {
                setHoodPos(highestValue);
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

    public void setGateClosed() {
        gate.setPosition(gateClosed);
    }

    public void setGateOpen() {
        gate.setPosition(gateOpen);
    }

    public double getShooterVelocity() {
        return primaryShooter.getVelocity() * -1;
    }

    public double getShooterPower() {
        return primaryShooter.getPower();
    }

    public double getHoodPos() {
        return hoodLeft.getPosition();
    }
}
