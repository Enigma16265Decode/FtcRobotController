package org.firstinspires.ftc.teamcode.teleopClasses;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.IndicatorColors;


enum ShootingRanges {
    CLOSE,
    FAR
}
public class Shooter {
    private int closeSpeed = 1100, farSpeed = 1460;
    private double closeHood = 0.85, farHood = 0.4;
    ShootingRanges shootingRange = ShootingRanges.CLOSE;
    private final Gamepad gamepad1;
    private final HardwareMap hardwareMap;
    private Turret turret;
    private Kinematics kinematics;
    public static double sP = 0.02, sI = 0/*.35 /*0.72 */, sD = 0; //sP was 0.018, and sI was 0.35
    public static int targetSpeed = 1100;
    public double gateClosed = 0.07;
    public double gateOpen = 0.9;
    private boolean shootToggle = false;
    private boolean gateToggle = false;
    private boolean shooterStopped = false;
    private PIDController shooterController;
    private DcMotorEx primaryShooter;
    private DcMotor secondaryShooter;
    private Servo hoodLeft;
    private Servo gate;
    private Servo indicator;
    private IndicatorColors indicatorColor = IndicatorColors.RED;
    private double rgb;


    public Shooter(@NonNull HardwareMap hardwareMap, Gamepad gamepad1, Turret turret, Kinematics kinematics) {
        shooterController = new PIDController(sP, sI, sD);

        indicator = hardwareMap.get(Servo.class, "indicator");
        primaryShooter = hardwareMap.get(DcMotorEx.class, "leftShooter"); //change depending on side
        secondaryShooter = hardwareMap.get(DcMotor.class, "rightShooter");
        hoodLeft = hardwareMap.get(Servo.class, "leftHood");
        gate = hardwareMap.get(Servo.class, "gate");


        secondaryShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        hoodLeft.setDirection(Servo.Direction.REVERSE);

        this.gamepad1 = gamepad1;
        this.hardwareMap = hardwareMap;
        this.turret = turret;
        this.kinematics = kinematics;
    }

    public boolean isShooterAtSpeed() {
        if((getShooterVelocity() > 1040) && (getShooterVelocity() < 1120)) {
            return true;
        }
        else {
            return false;
        }
    }

    public void runRgb(IndicatorColors color) {
        indicatorColor = color;
        if(indicatorColor == null) {
            indicator.setPosition(0.277);
        }
        if(indicatorColor == IndicatorColors.GREEN) {
            indicator.setPosition(0.485);
        }
        if(indicatorColor == IndicatorColors.RED) {
            indicator.setPosition(0.277);
        }
        if(indicatorColor == IndicatorColors.RAINBOW) {
            if(rgb >= 0.722) {
                rgb = 0.277;
            }
            else {
                rgb += 0.001;
            }
            indicator.setPosition(rgb);
        }
    }

    public void setRgbBasedOnDistance() {
        double ideal = 67;
        double off = 6;
        double max = ideal + off;
        double min = ideal - off;
        double distance = kinematics.getDistanceToGoal();
        if(distance < max && distance > min) {
            runRgb(IndicatorColors.GREEN);
        }
        else {
            runRgb(IndicatorColors.RED);
        }
    }



    public void setShooterPower(double value) {
        primaryShooter.setPower(value);
        secondaryShooter.setPower(value);
    }

    public void toggleShootingRange() {
        ShootingRanges initialState = shootingRange;
        if (gamepad1.xWasPressed()) {
            if(initialState == ShootingRanges.CLOSE) {
                shootingRange = ShootingRanges.FAR;
            }
            if(initialState == ShootingRanges.FAR) {
                shootingRange = ShootingRanges.CLOSE;
            }
            setHardwareShootingState();
        }
    }

    public void setHardwareShootingState() {
        if(shootingRange == ShootingRanges.CLOSE) {
            targetSpeed = closeSpeed;
            setHoodPos(closeHood);
        }
        if(shootingRange == ShootingRanges.FAR) {
            targetSpeed = farSpeed;
            setHoodPos(farHood);
        }
    }

    public ShootingRanges getShootingRange() {
        return shootingRange;
    }

    public void initHood() {
        setHoodPos(0.9);
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

    int presses = 0;
    public void gateController() {
        if(gamepad1.bWasPressed() && gamepad1.b) {
            toggleGate();
            presses += 1;
        }
    }
    public int getPresses() {
        return presses;
    }

    public void setShooterStopped(boolean stop) {
        shooterStopped = stop;
    }


    private void setHoodPos(double value) {
        hoodLeft.setPosition(value);
    }

    public void hoodControl() {
        final double lowestValue = 0.0;
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

    public void setHoodForClose() {
        hoodLeft.setPosition(closeHood);
    }

    public void toggleGate() {
        double initialState = gate.getPosition();

        if(initialState == gateClosed) {
            gate.setPosition(gateOpen);
        }
        if(initialState == gateOpen) {
            gate.setPosition(gateClosed);
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

    public String getShootingRangeString() {
        if(shootingRange == ShootingRanges.CLOSE) {
            return "Close";
        }
        if (shootingRange == ShootingRanges.FAR) {
            return "Far";
        }
        return null;
    }

    public double getShooterPower() {
        return primaryShooter.getPower();
    }

    public double getHoodPos() {
        return hoodLeft.getPosition();
    }
    public double getGatePos() {
        return gate.getPosition();
    }
}
