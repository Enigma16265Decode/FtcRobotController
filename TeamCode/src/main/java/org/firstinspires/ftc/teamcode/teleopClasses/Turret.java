package org.firstinspires.ftc.teamcode.teleopClasses;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    private HardwareMap hardwareMap;
    private Kinematics kinematics;
    private DcMotorEx turret;
    public static double kP = 0.0, kI = 0.0, kD = 0.0;
    PIDController turretController = new PIDController(kP, kI, kD);
    public double targetPos = 100.0;
    private double ticksPerDegree = 2.0; //Todo: find this
    public Turret(HardwareMap hardwareMap, Kinematics kinematics) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");

        this.hardwareMap = hardwareMap;
        this.kinematics = kinematics;
    }

    public void moveTurret() {
        double currentVelocity = turret.getVelocity();

        turretController.setPID(kP, kI, kD);
        double turretPid = turretController.calculate(currentVelocity, targetPos);

        turret.setPower(turretPid);
    }

    public void setTargetBasedOnHeadingToGoal() {
        targetPos = kinematics.getHeadingToGoal() * ticksPerDegree;
    }
}
