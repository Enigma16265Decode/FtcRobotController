package org.firstinspires.ftc.teamcode.tuners;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Configurable
@TeleOp(name = "Turret Tuner", group = "tuners")
public class TurretPIDTuner extends OpMode {
    private TelemetryManager telemetryM;
    private DcMotorEx turret;
    private static double offset;
    public static double kP = 0.0,  kI = 0.0, kD = 0.0; //0.002, 0.00007
    public static double f = 0; //0.025
    PIDController turretController = new PIDController(kP, kI, kD);
    public static double targetPos = 100.0;
    private static double ticksInDegree = 4100.0 / 180.0;


    private void runPID() {
        double currentPosition = turret.getCurrentPosition();

        turretController.setPID(kP, kI, kD);
        double currentPos = currentPosition - offset;
        double turretPid = turretController.calculate(currentPos, targetPos);
        double ff;
        if(targetPos > currentPosition) {
            ff = Math.cos(Math.toRadians(targetPos / ticksInDegree)) * f;
        }
        else {
            ff = Math.cos(Math.toRadians(targetPos / ticksInDegree)) * f * -1;
        }

        double power = turretPid + ff;

        turret.setPower(power);
    }

    @Override
    public void init() {
        turretController = new PIDController(kP, kI, kD);
        turretController.setTolerance(0.5);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        turret = hardwareMap.get(DcMotorEx.class, "turret");
    }
    @Override
    public void start() {

    }

    @Override
    public void loop() {
        runPID();

        telemetryM.addData("target", targetPos);
        telemetryM.addData("current pos true", turret.getCurrentPosition());
        telemetryM.addData("current pos", turret.getCurrentPosition() - offset);
        telemetryM.addData("velocity", turret.getVelocity());

        telemetryM.update();
    }
}
