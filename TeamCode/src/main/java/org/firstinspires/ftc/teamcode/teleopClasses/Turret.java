package org.firstinspires.ftc.teamcode.teleopClasses;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Alliances;

public class Turret {
    private HardwareMap hardwareMap;
    private Kinematics kinematics;
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private Alliances alliance;
    private Servo turret1, turret2;
    private static double kP = 0.002, kI = 0.0, kD = 0.00007; //0.04 & 0.0015
    public static double f = 0.025;
    PIDController turretController = new PIDController(kP, kI, kD);
    private double targetPos = 0.0;
    private double ticksInDegree = 16265.0 / 9080.0; //todo measure
    private boolean homeOverride = false;
    public Turret(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Kinematics kinematics, Alliances alliance) {
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret1 = hardwareMap.get(Servo.class, "turret2");
        turret2.setDirection(Servo.Direction.REVERSE);
        //turretController.setTolerance(0.5);

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.hardwareMap = hardwareMap;
        this.kinematics = kinematics;
        this.alliance = alliance;
        this.homeOverride = false;
    }

    public Turret(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Kinematics kinematics, boolean homeOverride) {
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret1 = hardwareMap.get(Servo.class, "turret2");
        turret2.setDirection(Servo.Direction.REVERSE);
        //turretController.setTolerance(0.5);

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.hardwareMap = hardwareMap;
        this.kinematics = kinematics;
        this.homeOverride = homeOverride;
    }

    /** Moves the turret according to PID and target pos**/
    public void moveTurret() {
        if(homeOverride) {
            targetPos = 0.5;
        }
        setTurretPosition(targetPos);
    }

    private void setTurretPosition(double position) {
        turret1.setPosition(position);
        turret2.setPosition(position);
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
