package org.firstinspires.ftc.teamcode.teleopClasses;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.enums.Alliances;
import org.firstinspires.ftc.teamcode.enums.LimelightStates;

public class Turret {
    private HardwareMap hardwareMap;
    private Kinematics kinematics;
    private LimelightSS limelight;
    private Alliances alliance;
    private Servo turretLeft, turretRight;
    private Timer limelightTimer;
    private double targetPos = 0.0; //0.59/180
    private double ticks = 0.56;
    private double degrees = 180;
    private double turretOffset = 0, limelightOffset = 0;
    private final double ticksInDegree = (double) ticks / degrees; //private final double ticksInDegree = (double) 1 / 330; 0.574 / 180
    private double turretZero = 0.5; //also 180
    private double turretZeroOffset = ticks - turretZero;
    private boolean homeOverride = false;
    public Turret(HardwareMap hardwareMap, Kinematics kinematics, LimelightSS limelight, Alliances alliance) {
        turretLeft = hardwareMap.get(Servo.class, "turretLeft");
        turretRight = hardwareMap.get(Servo.class, "turretRight"); //0.782 0.222
        turretRight.setDirection(Servo.Direction.REVERSE);
        //turretController.setTolerance(0.5);
        limelightTimer = new Timer();
        limelightTimer.resetTimer();

        this.hardwareMap = hardwareMap;
        this.kinematics = kinematics;
        this.limelight = limelight;
        this.alliance = alliance;
        this.homeOverride = false;
    }

    /** Moves the turret according to PID and target pos**/
    public void moveTurret() {
        limelight.limelightController();

        final double turretSpeed = limelight.getTx() * ticksInDegree;


        /*
        if(getLimelightState() != LimelightStates.LOST && getLimelightState() != LimelightStates.NON_VALID_TARGET) {
            if(limelightTimer.getElapsedTime() > 850) {
                if(limelight.getLimelightState() == LimelightStates.OFF_RIGHT) {
                    targetPos = turretLeft.getPosition() + turretSpeed;
                }
                if(limelight.getLimelightState() == LimelightStates.OFF_LEFT) {
                    targetPos = turretLeft.getPosition() + turretSpeed;
                }
                limelightTimer.resetTimer();
            }
        }
        else {
            setTargetBasedOnHeadingToGoal();
        }

         */

        if(limelight.getTx() > -9 && limelight.getTx() < 9) {
            limelightOffset = limelight.getTx() * ticksInDegree * -1;
        }


        setTargetBasedOnHeadingToGoal();


        if(homeOverride) {
            targetPos = 0.5;
        }


        double totalOffset = turretOffset + limelightOffset;
        setTurretPosition(targetPos + totalOffset);
    }

    public double getLimelightOffset() {
        return limelightOffset;
    }

    public void offsetController(Gamepad gamepad2) {
        final double toOffset = 0.0075;
        if(gamepad2.leftBumperWasPressed()) {
            turretOffset += toOffset;
        }
        if(gamepad2.rightBumperWasPressed()) {
            turretOffset -= toOffset;
        }
        if(gamepad2.backWasPressed()) {
            homeOverride = true;
        }
    }


    public LimelightStates getLimelightState() {
        return limelight.getLimelightState();
    }


    public boolean turretReadyToShoot() {
        /*
        if(limelight.getLimelightState() == LimelightStates.CENTERED) {
            return true;
        }
        else {
            return false;
        }

         */
        return true;
    }

    private void setTurretPosition(double position) {
        final double offset = -0.03;
        if(position > 1) {
            position = 1;
        }
        if(position < 0.18) {
            position = 0.18;
        }
        turretLeft.setPosition(position);
        turretRight.setPosition(position + offset);
    }

    public double getCurrentPos() {
        return turretLeft.getPosition();
    }

    public double getTargetPos() {
        return targetPos;
    }



    /** Sets the turret's target based on robot position relative to goal **/
    public void setTargetBasedOnHeadingToGoal() {
        if(!homeOverride) {
            targetPos = (kinematics.getHeadingToGoal() * ticksInDegree) - turretZeroOffset;
        }
    }
}
