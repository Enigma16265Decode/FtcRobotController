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
    private final double ticks = 0.5886; //0.56;
    private final double degrees = 180;
    private double turretOffset = 0, limelightOffset = 0;
    private final double ticksInDegree = (double) ticks / degrees; //private final double ticksInDegree = (double) 1 / 330; 0.574 / 180
    private final double turretZero = 0.4968; //also 180
    private final double turretZeroOffset = ticks - turretZero;
    private boolean turretOverride = false;
    private double turretOverrideValue = 0;
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
    }

    /** Moves the turret according to PID and target pos**/
    public void moveTurret() {
        limelight.limelightController();

        /*

        final double turretSpeed = limelight.getTx() * ticksInDegree;
        double suggestedOffset = 0;

        if(getLimelightState() != LimelightStates.LOST && getLimelightState() != LimelightStates.NON_VALID_TARGET) {
            if(limelightTimer.getElapsedTime() > 1000) {
                if(limelight.getLimelightState() == LimelightStates.OFF_RIGHT) {
                    suggestedOffset += turretSpeed;
                }
                if(limelight.getLimelightState() == LimelightStates.OFF_LEFT) {
                    suggestedOffset -= turretSpeed;
                }
                limelightTimer.resetTimer();
            }
        }
        else {
            setTargetBasedOnHeadingToGoal();
        }



        if(limelight.getTx() > -4 && limelight.getTx() < 4) {
            limelightOffset = suggestedOffset;
        }

        */


        setTargetBasedOnHeadingToGoal();

        if(!turretOverride) {
            double totalOffset = turretOffset + limelightOffset;
            setTurretPosition(targetPos + totalOffset);
        }
        else {
            setTurretPosition(turretOverrideValue);
        }
    }

    public void toggleTurretOverride() {
        boolean prevState = turretOverride;
        turretOverride = !prevState;
    }

    public void setTurretOverride(boolean toSet) {
        turretOverride = toSet;
    }

    public void setTurretOverride(boolean toSet, double value) {
        turretOverride = toSet;
        turretOverrideValue = value;
    }

    public void setTurretOverride(double value) {
        turretOverrideValue = value;
    }

    public double getLimelightOffset() {
        return limelightOffset;
    }

    public void offsetController(Gamepad gamepad2) {
        final double toOffset = 0.02;
        if(gamepad2.leftBumperWasPressed()) {
            turretOffset += toOffset;
        }
        if(gamepad2.rightBumperWasPressed()) {
            turretOffset -= toOffset;
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
        final double offset = -0.0;
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

    public double getTurretOffset() {
        return turretOffset;
    }

    public double getTargetPos() {
        return targetPos;
    }



    /** Sets the turret's target based on robot position relative to goal **/
    public void setTargetBasedOnHeadingToGoal() {
        if(!turretOverride) {
            targetPos = (kinematics.getHeadingToGoal() * ticksInDegree) - turretZeroOffset;
        }
    }
}
