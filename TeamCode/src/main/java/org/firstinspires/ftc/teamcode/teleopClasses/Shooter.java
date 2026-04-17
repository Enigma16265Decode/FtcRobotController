package org.firstinspires.ftc.teamcode.teleopClasses;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.enums.IntakeModes;

import java.util.HashMap;
import java.util.Map;


enum ShootingRanges {
    CLOSE,
    FAR
}
public class Shooter {
    private int closeSpeed = 1200, farSpeed = 1450;
    ShootingRanges shootingRange = ShootingRanges.CLOSE;
    private final Gamepad gamepad1;
    private final HardwareMap hardwareMap;
    private Turret turret;
    private Intake intake;
    private Kinematics kinematics;
    private Timer shootTimer;
    public static double sP = 0.02, sI = 0/*.35 /*0.72 */, sD = 0; //sP was 0.018, and sI was 0.35
    public static int targetSpeed = 1200;
    public double gateClosed = 0.3;
    public double gateOpen = 0.7;
    private boolean shootToggle = false;
    private boolean gateToggle = false;
    private int shootingState = -1;
    private PIDController shooterController;
    private DcMotorEx primaryShooter;
    private DcMotor secondaryShooter;
    private Servo hood;
    private Servo gate;
    private int[] closeShooterVelocities = {1100, 1130, 1200, 1250, 1300};
    private double[] closeHoodPoses = {0.4, 0.48, 0.58, 0.75, 0.81};
    private double[] closeDistances = {52, 64, 76, 88};
    Map<Double, Double> distanceFromHoodPos = new HashMap<>();

    public Shooter(@NonNull HardwareMap hardwareMap, Gamepad gamepad1, Turret turret, Intake intake, Kinematics kinematics) {
        shooterController = new PIDController(sP, sI, sD);

        primaryShooter = hardwareMap.get(DcMotorEx.class, "leftShooter"); //change depending on side
        secondaryShooter = hardwareMap.get(DcMotor.class, "rightShooter");
        hood = hardwareMap.get(Servo.class, "hood");
        gate = hardwareMap.get(Servo.class, "gate");


        secondaryShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        gate.setDirection(Servo.Direction.REVERSE);

        initializeMaps();

        this.gamepad1 = gamepad1;
        this.hardwareMap = hardwareMap;
        this.turret = turret;
        this.intake = intake;
        this.kinematics = kinematics;

        setHoodPos(0.76);
    }



    /** Adds values into maps **/
    private void initializeMaps() {
        distanceFromHoodPos.put(closeHoodPoses[0], closeDistances[0]);
        distanceFromHoodPos.put(closeHoodPoses[1], closeDistances[1]);
        distanceFromHoodPos.put(closeHoodPoses[2], closeDistances[2]);
    }

    /*
    public void shootControl() {
        if(gamepad1.bWasPressed()) {
            if(turret.turretReadyToShoot() && isShooterAtSpeed()) {
                shoot();
            }
        }
    }

     */

    public void shoot() {
        if(shootingState == -1) {
            shootingState = 0;
        }
        tryShooting();
    }


    public void tryShooting() {
        switch (shootingState) {
            case 0:
                shootTimer.resetTimer();
                setGateOpen();
                shootingState = 1;
                break;
            case 1:
                if(shootTimer.getElapsedTime() >= 300) {
                    intake.setIntakeMode(IntakeModes.SHOOT);
                    shootTimer.resetTimer();
                    shootingState = 2;
                }
                break;
            case 2:
                if(shootTimer.getElapsedTime() >= 1700) {
                    intake.setIntakeMode(IntakeModes.IDLE);
                    setGateClosed();
                    shootingState = -1;
                }
                break;
        }
    }


    /** Checks if x is off by less than y amount in either direction (positive or negative) **/
    public boolean betweenMinMaxWithTolerance(double x, double ideal, double off) {
        double max = ideal + off;
        double min = ideal - off;

        if(x < max && x > min) {
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

    /** Toggles shootingRange between ShootingRanges.CLOSE and ShootingRanges.FAR based on gamepad input **/
    public void toggleShootingRange() {
        ShootingRanges initialState = shootingRange;
        if (gamepad1.aWasPressed()) {
            if(initialState == ShootingRanges.CLOSE) {
                shootingRange = ShootingRanges.FAR;
            }
            if(initialState == ShootingRanges.FAR) {
                shootingRange = ShootingRanges.CLOSE;
            }
        }
    }

    /** Returns an int based on robot's distance to the goal**/
    private int getDistanceIndex() {
        //double ideal = 64; // was 67
        double off = 6;
        double distance = kinematics.getDistanceToGoal();
        if(betweenMinMaxWithTolerance(distance, closeDistances[0], off)) {
            return 0;
        }
        if(betweenMinMaxWithTolerance(distance, closeDistances[1], off)) {
            return 1;
        }
        if(betweenMinMaxWithTolerance(distance, closeDistances[2], off)) {
            return 2;
        }
        if(betweenMinMaxWithTolerance(distance, closeDistances[3], off)) {
            return 3;
        }
        return 4;
    }

    private void setShooterVelocityBasedOnMode() {
        if(shootingRange == ShootingRanges.CLOSE) {
            targetSpeed = closeShooterVelocities[getDistanceIndex()];
        }
        else {
            targetSpeed = farSpeed;
        }
    }

    private double hoodPosBasedOnDistance() {
        return closeHoodPoses[getDistanceIndex()];
    }


    public void shooterController() {
        setShooterVelocityBasedOnMode();
        //setHoodPos(hoodPosBasedOnDistance());


        if(gamepad1.bWasPressed()) {
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
            runShooterPID();
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

    public void runShooterPID() {
        double currentVelocity = primaryShooter.getVelocity();

        shooterController.setPID(sP, sI, sD);
        double shooterPid = shooterController.calculate(currentVelocity, targetSpeed);

        if(shooterPid < -0.2 && shootingRange == ShootingRanges.CLOSE) {
            shooterPid = -0.2;
        }
        setShooterPower(shooterPid);
    }
    public void turretController() {
        turret.setTargetBasedOnHeadingToGoal();
        turret.moveTurret();
    }




    public void setHoodPos(double value) {
        hood.setPosition(value);
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
        return primaryShooter.getVelocity();
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
        return hood.getPosition();
    }
    public double getGatePos() {
        return gate.getPosition();
    }
}
