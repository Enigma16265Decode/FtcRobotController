package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "Shooter PID tuner")
public class PIDTuner extends LinearOpMode {
    public static double sP = 0.0003, sI = 0.7 /*0.72 */, sD = 0; //we will almost certainly not change d, change p after finding good i value
    public static int targetSpeed = 1200; //11 / 13.6 * 1480
    private final double ticksInDegree = 0;

    static TelemetryManager telemetryM;
    PIDController shooterController;

    private DcMotorEx primaryShooter;
    private DcMotor secondaryShooter;
    private Servo hoodLeft;
    private Servo hoodRight;

    private void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        shooterController = new PIDController(sP, sI, sD);
        //maybe do something with telemetry I really don't know im way underqualified

        primaryShooter = hardwareMap.get(DcMotorEx.class, "leftShooter"); //change depending on side
        secondaryShooter = hardwareMap.get(DcMotor.class, "rightShooter");
        hoodLeft = hardwareMap.get(Servo.class, "leftHood");
        hoodRight = hardwareMap.get(Servo.class, "rightHood");

        hoodLeft.setDirection(Servo.Direction.REVERSE);

        primaryShooter.setDirection(DcMotorSimple.Direction.REVERSE); //again, same as above ^
    }
    private void masterFunction() {
        double currentVelocity = primaryShooter.getVelocity();

        if(gamepad1.a) {
            shooterController.setPID(sP, sI, sD);
            double shooterPid = shooterController.calculate(currentVelocity, targetSpeed);

            primaryShooter.setPower(shooterPid);
            secondaryShooter.setPower(shooterPid);
        }
        else {
            primaryShooter.setPower(0);
            secondaryShooter.setPower(0);
        }

        telemetryM.debug("velocity: ", currentVelocity);
        telemetryM.debug("target :", targetSpeed);
        telemetryM.debug("hood pos: ", hoodLeft.getPosition());
        telemetryM.update();

        telemetry.addData("velocity: ", currentVelocity);
        telemetry.addData("target :", targetSpeed);
        telemetry.addData("hood pos: ", hoodLeft.getPosition());
        telemetry.update();
        hoodControl();
    }

    private void setHoodPos(double value) {
        hoodLeft.setPosition(value);
        hoodRight.setPosition(value);
    }

    private void hoodControl() {
        double amountToMove = 0.05;

        if(gamepad1.left_bumper && gamepad1.leftBumperWasPressed()) {
            double toSet = (hoodLeft.getPosition() - amountToMove);
            if (toSet < 0.2) {
                setHoodPos(0.2);
            }
            else {
                setHoodPos(toSet);
            }
        }
        if(gamepad1.right_bumper && gamepad1.rightBumperWasPressed()) {
            double toSet = (hoodLeft.getPosition() + amountToMove);

            if (toSet > 0.9) {
                setHoodPos(0.9);
            }
            else {
                setHoodPos(toSet);
            }
        }
    }

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        //run this code
        //I hate kotlin >:-(

        while (opModeIsActive()) {
            masterFunction();
            /*
            if(gamepad1.x) {
                primaryShooter.setPower(0.5);
            }
            if(gamepad1.b) {
                secondaryShooter.setPower(0.5);
            }



            primaryShooter.setPower(1);
            secondaryShooter.setPower(1); //IMPORTANT NEEDS TO BE AT MAX VOLTAGE WHEN RUNNING TEST!!!

            telemetryM.debug("velocity: ", primaryShooter.getVelocity());
            telemetryM.update();

             */
        }
    }
}
