package org.firstinspires.ftc.teamcode.tuners;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "ShooterPIDTuner", group = "tuners")
public class ShooterPIDTuner extends OpMode {
    private TelemetryManager telemetryM;
    private DcMotorEx primaryShooter;
    private DcMotor secondaryShooter;
    private Servo hood;
    private Servo gate;
    public static double sP = 0.02, sI = 0.0, sD = 0.0;
    PIDController shooterController = new PIDController(sP, sI, sD);
    private static double targetSpeed = 1100.0;
    private static double hoodPos = 0.25;
    private static double currentVelForPanel = 0;


    private void runPID() {
        double currentVelocity = primaryShooter.getVelocity();


        shooterController.setPID(sP, sI, sD);
        double shooterPid = shooterController.calculate(currentVelocity, targetSpeed);

        setShooterPower(shooterPid);
    }

    private void setShooterPower(double value) {
        primaryShooter.setPower(value);
        secondaryShooter.setPower(value);
    }

    private void setHoodPosition() {
        hood.setPosition(hoodPos);
    }

    @Override
    public void init() {
        shooterController = new PIDController(sP, sI, sD);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        shooterController = new PIDController(sP, sI, sD);

        primaryShooter = hardwareMap.get(DcMotorEx.class, "leftShooter"); //change depending on side
        secondaryShooter = hardwareMap.get(DcMotor.class, "rightShooter");

        gate = hardwareMap.get(Servo.class, "gate");
        hood = hardwareMap.get(Servo.class, "hood");

        hood.setDirection(Servo.Direction.REVERSE);
        secondaryShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        gate.setPosition(hoodPos);
    }
    @Override
    public void start() {

    }

    @Override
    public void loop() {
        runPID();
        setHoodPosition();

        telemetryM.debug("target", targetSpeed);
        telemetryM.debug("velocity", primaryShooter.getVelocity());
        currentVelForPanel = primaryShooter.getVelocity();

        telemetryM.update();

        telemetry.addData("target", targetSpeed);
        telemetry.addData("velocity", primaryShooter.getVelocity());

        telemetry.update();
    }
}
