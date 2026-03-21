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

import org.firstinspires.ftc.teamcode.enums.IndicatorColors;
import org.firstinspires.ftc.teamcode.enums.IntakeModes;

import java.util.HashMap;
import java.util.Map;


enum ShootingRanges {
    CLOSE,
    FAR
}
public class Shooter {
    private int closeSpeed = 1100, farSpeed = 1350;
    private double closeHood = 0.85, farHood = 0.4;
    ShootingRanges shootingRange = ShootingRanges.CLOSE;
    private final Gamepad gamepad1;
    private final HardwareMap hardwareMap;
    private Turret turret;
    private Intake intake;
    private Kinematics kinematics;
    private Timer shootTimer;
    public static double sP = 0.02, sI = 0/*.35 /*0.72 */, sD = 0; //sP was 0.018, and sI was 0.35
    public static int targetSpeed = 1100;
    public double gateClosed = 0.07;
    public double gateOpen = 0.9;
    private boolean shootToggle = false;
    private boolean gateToggle = false;
    private boolean shooterStopped = false;
    private int shootingState = -1;
    private PIDController shooterController;
    private DcMotorEx primaryShooter;
    private DcMotor secondaryShooter;
    private Servo hoodLeft;
    private Servo gate;
    private Servo indicator;
    private int[] closeVelocities = {1100, 1170, 1270, 1170};
    private double[] closeDistances = {64, 76, 88};
    Map<IndicatorColors, Double> indicatorColorsDouble = new HashMap<>();
    Map<Integer, Double> distanceFromVelocity = new HashMap<>();

    public Shooter(@NonNull HardwareMap hardwareMap, Gamepad gamepad1, Turret turret, Intake intake, Kinematics kinematics) {
        shooterController = new PIDController(sP, sI, sD);

        indicator = hardwareMap.get(Servo.class, "indicator");
        primaryShooter = hardwareMap.get(DcMotorEx.class, "leftShooter"); //change depending on side
        secondaryShooter = hardwareMap.get(DcMotor.class, "rightShooter");
        hoodLeft = hardwareMap.get(Servo.class, "leftHood");
        gate = hardwareMap.get(Servo.class, "gate");


        secondaryShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        hoodLeft.setDirection(Servo.Direction.REVERSE);

        initializeMaps();

        this.gamepad1 = gamepad1;
        this.hardwareMap = hardwareMap;
        this.turret = turret;
        this.intake = intake;
        this.kinematics = kinematics;
    }

    /** Adds values into maps **/
    private void initializeMaps() {
        distanceFromVelocity.put(closeVelocities[0], closeDistances[0]);
        distanceFromVelocity.put(closeVelocities[1], closeDistances[1]);
        distanceFromVelocity.put(closeVelocities[2], closeDistances[2]);

        indicatorColorsDouble.put(IndicatorColors.RED, 0.282);
        indicatorColorsDouble.put(IndicatorColors.GREEN, 0.46);
        indicatorColorsDouble.put(IndicatorColors.BLUE, 0.61);
        indicatorColorsDouble.put(IndicatorColors.PURPLE, 0.7);
    }

    public void shootControl() {
        if(gamepad1.xWasPressed()) {
            if(turret.turretReadyToShoot() && isShooterAtSpeed()) {
                shoot();
            }
        }
    }

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

    public boolean isShooterAtSpeed() {
        //used by auto so should fine to be hard coded, also writing this comment where spur the moment, time is of the essence
        if((getShooterVelocity() > 1030) && (getShooterVelocity() < 1120)) {
            return true;
        }
        else {
            return false;
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
        setHoodPos(closeHood);
    }

    /** Sets the indicator light's color based on getDistanceIndex() **/
    public void setRgbBasedOnDistance() {
        if(getDistanceIndex() == 0) {
            indicator.setPosition(indicatorColorsDouble.get(IndicatorColors.GREEN));
        }
        if(getDistanceIndex() == 1) {
            indicator.setPosition(indicatorColorsDouble.get(IndicatorColors.BLUE));
        }
        if(getDistanceIndex() == 2) {
            indicator.setPosition(indicatorColorsDouble.get(IndicatorColors.PURPLE));
        }
        if(getDistanceIndex() == 3) {
            indicator.setPosition(indicatorColorsDouble.get(IndicatorColors.RED));
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
        return 3;
    }

    private int targetSpeedBasedOnDistance() {
        return closeVelocities[getDistanceIndex()];
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
        targetSpeed = targetSpeedBasedOnDistance();

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
    public void turretController() {
        turret.setTargetBasedOnHeadingToGoal();
        turret.moveTurret();
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

    public double getVelocityError() {
        double velocityError = targetSpeed - (primaryShooter.getVelocity()*-1);
        return velocityError;
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
