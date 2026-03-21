package org.firstinspires.ftc.teamcode.teleopClasses;

import com.arcrobotics.ftclib.controller.PIDController;
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
    private Servo turret1, turret2;
    private static double kP = 0.002, kI = 0.0, kD = 0.00007; //0.04 & 0.0015
    public static double f = 0.025;
    PIDController turretController = new PIDController(kP, kI, kD);
    private double targetPos = 0.0;
    private double ticksInDegree = 16265.0 / 9080.0; //todo measure
    private boolean homeOverride = false;
    public Turret(HardwareMap hardwareMap, Kinematics kinematics, LimelightSS limelight, Alliances alliance) {
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret1 = hardwareMap.get(Servo.class, "turret2");
        turret2.setDirection(Servo.Direction.REVERSE);
        //turretController.setTolerance(0.5);

        this.hardwareMap = hardwareMap;
        this.kinematics = kinematics;
        this.limelight = limelight;
        this.alliance = alliance;
        this.homeOverride = false;
    }

    public Turret(HardwareMap hardwareMap, Kinematics kinematics, LimelightSS limelight, boolean homeOverride) {
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret1 = hardwareMap.get(Servo.class, "turret2");
        turret2.setDirection(Servo.Direction.REVERSE);
        //turretController.setTolerance(0.5);

        this.hardwareMap = hardwareMap;
        this.kinematics = kinematics;
        this.limelight = limelight;
        this.homeOverride = homeOverride;
    }

    /** Moves the turret according to PID and target pos**/
    public void moveTurret() {
        final double turretSpeed = 0.001;

        if(limelight.fiducialIsValidTarget()) {
            if(limelight.getLimelightState() == LimelightStates.OFF_RIGHT) {
                targetPos = turret1.getPosition() - turretSpeed;
            }
            if(limelight.getLimelightState() == LimelightStates.OFF_LEFT) {
                targetPos = turret1.getPosition() + turretSpeed;
            }
        }

        if(homeOverride) {
            targetPos = 0.5;
        }
        setTurretPosition(targetPos);
    }

    public boolean turretReadyToShoot() {
        if(limelight.getLimelightState() == LimelightStates.CENTERED) {
            return true;
        }
        else {
            return false;
        }
    }

    private void setTurretPosition(double position) {
        final double offset = 0;
        turret1.setPosition(position);
        turret2.setPosition(position + offset);
    }

    public double getCurrentPos() {
        return turret1.getPosition();
    }



    /** Sets the turret's target based on robot position relative to goal **/
    public void setTargetBasedOnHeadingToGoal() {
        if(!homeOverride) {
            targetPos = (kinematics.getHeadingToGoal() * ticksInDegree);
        }
    }
}
