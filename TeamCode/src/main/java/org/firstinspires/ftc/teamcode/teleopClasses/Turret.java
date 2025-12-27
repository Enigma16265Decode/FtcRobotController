package org.firstinspires.ftc.teamcode.teleopClasses;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    private HardwareMap hardwareMap;
    private Kinematics kinematics;
    private DcMotorEx turret;
    public static double kP = 0.05, kI = 0.0, kD = 0.0015; //0.04 & 0.0015
    PIDController turretController = new PIDController(kP, kI, kD);
    public double targetPos = 100.0;
    private double posOnInit;
    private double ticksPerDegree = 316.0 / 180.0;
    private final double max = 158, min = -158; //158, but limiting for safety
    public Turret(HardwareMap hardwareMap, Kinematics kinematics) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        posOnInit = turret.getCurrentPosition();

        this.hardwareMap = hardwareMap;
        this.kinematics = kinematics;
    }

    public void moveTurret() {
        turretController.setPID(kP, kI, kD);
        double currentPos = turret.getCurrentPosition() - posOnInit;
        double turretPid = turretController.calculate(currentPos, targetPos);


        turret.setPower(turretPid);
    }

    public void setTargetBasedOnHeadingToGoal() {
        double toSet = kinematics.getHeadingToGoal() * ticksPerDegree;
        if (toSet < min) {
            toSet = min;
        }
        if (toSet > max) {
            toSet = max;
        }
        targetPos = toSet;
    }
}
