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
    public static double kP = 0.05, kI = 0.0, kD = 0.0007; //0.04 & 0.0015
    PIDController turretController = new PIDController(kP, kI, kD);
    public double targetPos = 0.0;
    private double posOnInit;
    private boolean homeOverride = false;
    private double offset = 0;
    public final double toOffset = 5;
    private double ticksPerDegree = 316.0 / 180.0;
    private final double max = 158, min = -158; //158, but limiting for safety
    public Turret(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Kinematics kinematics, boolean homeOverride) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        posOnInit = turret.getCurrentPosition();

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.hardwareMap = hardwareMap;
        this.kinematics = kinematics;
        this.homeOverride = homeOverride;
    }


    public void moveTurret() {
        turretController.setTolerance(0.5);
        turretController.setPID(kP, kI, kD);
        double currentPos = turret.getCurrentPosition() - posOnInit;
        double turretPid = turretController.calculate(currentPos, targetPos);


        turret.setPower(turretPid);
    }

    public double getOffset() {
        return offset;
    }

    public void setOffset() {
        if(gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed()) {
            offset -= toOffset;
        }
        if(gamepad1.dpadRightWasPressed() || gamepad2.dpadLeftWasPressed()) {
            offset += toOffset;
        }
    }


    public void setTargetBasedOnHeadingToGoal(boolean isRed) {
        if(!homeOverride) {
            double toSet = (kinematics.getHeadingToGoal(isRed) * ticksPerDegree) - offset;
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
