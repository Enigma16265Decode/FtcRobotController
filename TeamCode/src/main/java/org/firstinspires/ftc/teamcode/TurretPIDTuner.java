package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.teleopClasses.Kinematics;

@Configurable
@TeleOp(name = "turret pid guy")
public class TurretPIDTuner extends OpMode {
    private TelemetryManager telemetryM;
    private DcMotorEx turret;
    private double posOnInit;
    public static double kP = 0.05, kI = 0.0, kD = 0.0015;
    public static double c = 1;
    PIDController turretController = new PIDController(kP, kI, kD);
    public static double targetPos = 100.0;
    private static double ticksInDegree = 316.0 / 180.0;


    public void moveTurret() {
        double currentVelocity = turret.getVelocity();

        turretController.setPID(kP, kI, kD);
        double turretPid = turretController.calculate(currentVelocity, targetPos);

        turret.setPower(turretPid);
    }

    public void setTarget(double toSet) {
        double max = 10000.0;
        double min = -10000.0;

        if (toSet < min) {
            toSet = min;
        }
        if (toSet > max) {
            toSet = max;
        }
        targetPos = toSet;
    }

    private void runPID() {
        turretController.setPID(kP, kI, kD);
        double currentPos = turret.getCurrentPosition() - posOnInit;
        double turretPid = turretController.calculate(currentPos, targetPos) * c;


        turret.setPower(turretPid);
    }

    @Override
    public void init() {
        turretController = new PIDController(kP, kI, kD);
        turretController.setTolerance(3);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        posOnInit = turret.getCurrentPosition();
    }
    @Override
    public void start() {

    }

    @Override
    public void loop() {
        runPID();

        telemetryM.debug("target", targetPos);
        telemetryM.debug("current pos", turret.getCurrentPosition() - posOnInit);
        telemetryM.debug("velocity", turret.getVelocity());
        telemetryM.debug("claimed pos error", turretController.getPositionError());

        telemetryM.update();

        telemetry.addData("target", targetPos);
        telemetry.addData("current pos", turret.getCurrentPosition() - posOnInit);
        telemetry.addData("velocity", turret.getVelocity());

        telemetry.update();
    }
}
