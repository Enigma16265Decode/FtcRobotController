package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "! FC TeleOP \uD83D\uDC80")
public class BasicTeleOp extends OpMode {
    private Follower follower;
    public static Pose startingPose = new Pose(14,14,Math.toRadians(0));
    private Supplier<PathChain> pathChain;

    public static double sP = 0.01, sI = 0.3 /*0.72 */, sD = 0; //we will almost certainly not change d, change p after finding good i value
    public static int targetSpeed = 1300; //11 / 13.6 * 1480
    private final double ticksInDegree = 0;
    double gateClosed = 0.4;
    double gateOpen = 0.2;

    static TelemetryManager telemetryM;
    PIDController shooterController;

    private DcMotorEx primaryShooter;
    private DcMotor secondaryShooter;
    private DcMotor intake;
    private DcMotor rightFront, rightRear, leftRear, leftFront;
    private Servo hoodLeft;
    private Servo hoodRight;
    private Servo gate;


    private void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        shooterController = new PIDController(sP, sI, sD);
        //maybe do something with telemetry I really don't know im way underqualified

        primaryShooter = hardwareMap.get(DcMotorEx.class, "leftShooter"); //change depending on side
        secondaryShooter = hardwareMap.get(DcMotor.class, "rightShooter");
        hoodLeft = hardwareMap.get(Servo.class, "leftHood");
        hoodRight = hardwareMap.get(Servo.class, "rightHood");
        intake = hardwareMap.get(DcMotor.class, "intake");
        gate = hardwareMap.get(Servo.class, "gate");

        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");

        primaryShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        secondaryShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        hoodLeft.setDirection(Servo.Direction.REVERSE);

        primaryShooter.setDirection(DcMotorSimple.Direction.REVERSE); //again, same as above ^

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    private void drivePOV() {
        double driveSpeed = 1;
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x/2;
        double strafe = gamepad1.left_stick_x;

        double lFront = drive + turn + strafe;
        double lBack = drive + turn - strafe;
        double rFront = drive - turn - strafe;
        double rBack = drive - turn + strafe;

        if(drive < 0.05) { drive = 0; } //I think this improves speed -E
        if(strafe < 0.05) { strafe = 0; }

        leftFront.setPower(lFront * driveSpeed);
        leftRear.setPower(lBack * driveSpeed);
        rightFront.setPower(rFront * driveSpeed);
        rightRear.setPower(rBack * driveSpeed);
    }

    private void setShooterPower(double value) {
        primaryShooter.setPower(value);
        secondaryShooter.setPower(value);
    }

    private void shooterController() {
        double currentVelocity = primaryShooter.getVelocity();

        if(gamepad1.left_bumper) {
            shooterController.setPID(sP, sI, sD);
            double shooterPid = shooterController.calculate(currentVelocity, targetSpeed);

            setShooterPower(shooterPid);
            gate.setPosition(gateOpen);
        }
        else {
            setShooterPower(0);
            gate.setPosition(gateClosed);
        }
    }

    private void intakeController() {
        if(gamepad1.right_trigger > 0.4) {
            intake.setPower(1);
        }
        else {
            if(gamepad1.left_trigger > 0.4) {
                intake.setPower(-1);
            }
            else {
                intake.setPower(0);
            }
        }
        /*
        if(!gamepad1.a && gamepad1.right_trigger > 0.4) {
            setShooterPower(-0.4);
        }

         */
    }

    private void telemetry() {
        //telemetryM.debug("velocity: ", currentVelocity);
        telemetryM.debug("target :", targetSpeed);
        telemetryM.debug("hood pos: ", hoodLeft.getPosition());
        telemetryM.update();

        //telemetry.addData("velocity: ", currentVelocity);
        telemetry.addData("target :", targetSpeed);
        telemetry.addData("hood pos: ", hoodLeft.getPosition());
        telemetry.addData("power : ", primaryShooter.getPower());
        telemetry.addData("shooter vel: ", primaryShooter.getVelocity());
        telemetry.addData("intake power: ", intake.getPower());
        telemetry.addData("gate pos: ", gate.getPosition());

        telemetry.update();
    }



    private void gateController() {
        if(gamepad1.x && gamepad1.xWasPressed()) {
            toggleGate();
        }
    }

    void toggleGate() {
        boolean hasToggled = false;

        if(gate.getPosition() == gateOpen && hasToggled == false) {
            gate.setPosition(gateClosed);
            hasToggled = true;
        }
        if(gate.getPosition() == gateClosed && hasToggled == false) {
            gate.setPosition(gateOpen);
            hasToggled = false;
        }
        else {
            gate.setPosition(gateClosed);
        }
    }

    private void setHoodPos(double value) {
        hoodLeft.setPosition(value);
        hoodRight.setPosition(value);
    }

    private void hoodControl() {
        double amountToMove = 0.05;

        if(gamepad1.dpad_right && gamepad1.dpadRightWasPressed()) {
            double toSet = (hoodLeft.getPosition() - amountToMove);
            if (toSet < 0.15) {
                setHoodPos(0.15);
            }
            else {
                setHoodPos(toSet);
            }
        }
        if(gamepad1.dpad_left && gamepad1.dpadLeftWasPressed()) {
            double toSet = (hoodLeft.getPosition() + amountToMove);

            if (toSet > 0.9) {
                setHoodPos(0.9);
            }
            else {
                setHoodPos(toSet);
            }
        }
    }

    private void fieldCentricDrive() {
        boolean automatedDrive = false;
        double slowmodeMultiplier = 0.3;

        //Call this once per loop
        follower.update();
        telemetryM.update();

        if (!automatedDrive) {


            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!gamepad1.left_bumper) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false // Robot Centric
            );
                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowmodeMultiplier,
                    -gamepad1.left_stick_x * slowmodeMultiplier,
                    -gamepad1.right_stick_x * slowmodeMultiplier,
                    false // Robot Centric
            );
        }



        //telemetryM.debug("position", follower.getPose());
        //telemetryM.debug("velocity", follower.getVelocity());
        //telemetryM.debug("automatedDrive", automatedDrive);
    }

    void relocalizePinpoint() {

    }

    private void masterFunction() {
        fieldCentricDrive();
        hoodControl();
        gateController();
        shooterController();
        intakeController(); //make sure this goes after shooter controller
        relocalizePinpoint();
        //drivePOV();


        telemetry();
    }

    /*
    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        setHoodPos(0.15);

        while (opModeIsActive()) {
            masterFunction();
        }
    }

     */

    //important im now gonna use opmode instead of linearopmode because i like it more

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        masterFunction();
    }

    @Override
    public void init() {
        initialize();
    }
}
