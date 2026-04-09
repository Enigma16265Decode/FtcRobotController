package org.firstinspires.ftc.teamcode.teleopClasses;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.enums.Alliances;
import org.firstinspires.ftc.teamcode.enums.LimelightStates;

public class Turret {
    private HardwareMap hardwareMap;
    private Kinematics kinematics;
    //private LimelightSS limelight;
    private Alliances alliance;
    private Servo turretLeft, turretRight;
    private static double kP = 0.002, kI = 0.0, kD = 0.00007; //0.04 & 0.0015
    public static double f = 0.025;
    PIDController turretController = new PIDController(kP, kI, kD);
    private double targetPos = 0.0;
    private double ticksInDegree = (double) 1 / 330; //todo measure
    private boolean homeOverride = false;
    public Turret(HardwareMap hardwareMap, Kinematics kinematics, /*LimelightSS limelight,*/ Alliances alliance) {
        turretLeft = hardwareMap.get(Servo.class, "turretLeft");
        turretRight = hardwareMap.get(Servo.class, "turretRight");
        turretRight.setDirection(Servo.Direction.REVERSE);
        //turretController.setTolerance(0.5);

        this.hardwareMap = hardwareMap;
        this.kinematics = kinematics;
        //this.limelight = limelight;
        this.alliance = alliance;
        this.homeOverride = false;
    }

    /** Moves the turret according to PID and target pos**/
    public void moveTurret() {
        final double turretSpeed = 0.001;

        /*
        if(limelight.fiducialIsValidTarget() && false) {
            if(limelight.getLimelightState() == LimelightStates.OFF_RIGHT) {
                targetPos = turretLeft.getPosition() - turretSpeed;
            }
            if(limelight.getLimelightState() == LimelightStates.OFF_LEFT) {
                targetPos = turretLeft.getPosition() + turretSpeed;
            }
        }

         */

        if(homeOverride) {
            targetPos = 0.5;
        }
        setTurretPosition(targetPos);
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
        final double offset = 0;
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
            targetPos = (kinematics.getHeadingToGoal() * ticksInDegree);
        }
    }
}
