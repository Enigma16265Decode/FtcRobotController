package org.firstinspires.ftc.teamcode.teleopClasses;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    private HardwareMap hardwareMap;
    private Kinematics kinematics;
    private DcMotorEx turret;
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private static double kP = 0.002, kI = 0.0, kD = 0.00007; //0.04 & 0.0015
    public static double f = 0.025;
    PIDController turretController = new PIDController(kP, kI, kD);
    private double targetPos = 0.0;
    private boolean homeOverride = false;
    private double offset = 0;
    private final double toOffset = 5;
    private static double ticksInDegree = 4100.0 / 180.0;
    private final double max = 2100, min = -2100;
    public Turret(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Kinematics kinematics, boolean homeOverride) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turretController.setTolerance(0.5);

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.hardwareMap = hardwareMap;
        this.kinematics = kinematics;
        this.homeOverride = homeOverride;
    }


    public void moveTurret() {
        /*
        turretController.setPID(kP, kI, kD);
        double currentPos = turret.getCurrentPosition() - offset;
        double turretPid = turretController.calculate(currentPos, targetPos);


        turret.setPower(turretPid);

         */

        double currentPosition = turret.getCurrentPosition();

        turretController.setPID(kP, kI, kD);
        double currentPos = currentPosition - offset;
        double turretPid = turretController.calculate(currentPos, targetPos);
        double ff;
        if(targetPos > currentPosition) {
            ff = Math.cos(Math.toRadians(targetPos / ticksInDegree)) * f * -1;
        }
        else {
            ff = Math.cos(Math.toRadians(targetPos / ticksInDegree)) * f;
        }

        double power = turretPid + ff;

        turret.setPower(power);
    }

    public double getOffset() {
        return offset;
    }

    public void setOffset() {
        if(gamepad1.dpadUpWasPressed() || gamepad2.dpadLeftWasPressed()) {
            offset -= toOffset;
        }
        if(gamepad1.dpadUpWasPressed() || gamepad2.dpadRightWasPressed()) {
            offset += toOffset;
        }
    }


    public void setTargetBasedOnHeadingToGoal(boolean isRed) {
        if(!homeOverride) {
            double toSet = (kinematics.getHeadingToGoal(isRed) * ticksInDegree) - offset;
            if (toSet < min) {
                toSet = min;
            }
            if (toSet > max) {
                toSet = max;
            }
            targetPos = toSet;
        }
    }
}
