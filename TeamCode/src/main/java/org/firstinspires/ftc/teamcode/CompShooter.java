package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/* The Shooter Class for the the varsity robot whatever
 * TODO: get the target shooter velocity. get the function of the angle
 * tetris pieces ░█
 *
 *
 *
 *
 */
@Configurable
@TeleOp(name = "TestCompShooter")
public class CompShooter extends LinearOpMode {
    enum RobotStates {
        IDLE,
        AIMING,
        READY_TO_SHOOT,
        SHOOTING,
        RELOCALIZING
    }

    RobotStates robotState = RobotStates.IDLE;

    Pose testPose1 = new Pose(14, 39, Math.toRadians(0));
    Pose testPose2 = new Pose(34, 19, Math.toRadians(0));

    DcMotorEx primaryShooter;
    DcMotor secondaryShooter;
    DcMotor rightFront, rightRear, leftRear, leftFront;
    Servo hoodLeft;
    Servo hoodRight;
    Servo turretLeft;
    Servo turretRight;
    Servo gate;
    GoBildaPinpointDriver pinpoint;

    double targetShooterVelocity = 11/16265; //note that the guide used as a reference used 10/13.5. replace 16265 with most likely 13.5-13.8
    //this could also be final I think ^
    public double p = 0, i = 0, d = 0; //we will almost certainly not change d, change p after finding good i value
    int maxVelocityVariability = 200;
    boolean isRed = true; //TODO I was working on getting the distance, but I needed the two goal poses and need them to be variable depending on which team you're on
    double distanceFromGoal;
    boolean aimingTurret = false;
    double driveSpeed = 1;
    boolean queueShooting = true;
    double turretOffset = 0.0;

    PIDController controller;
    double pid;
    Pose goalPose;
    Pose estimatedRobotPose;

    /**Called when determining if you can shoot*/
    boolean isReadyToShoot(boolean buttonPressed) {
        if (isShooterAtSpeed() && buttonPressed) { //TODO add other params here
            return true;
        }
        else {
            return false;
        }
    }

    /**Called within isReadyToShoot method*/
    boolean isShooterAtSpeed() {
        double max = targetShooterVelocity + maxVelocityVariability;
        double min = targetShooterVelocity - maxVelocityVariability;

        if (primaryShooter.getVelocity() <= max || primaryShooter.getVelocity() >= min ) {
            return true;
        }
        else {
            return false;
        }
    }

    /**Always being called to estimate the distance between the robot and the team specific goal*/ //
    private void updateDistance() {
        Pose2D estimatedRobotPose2D = pinpoint.getPosition();
        estimatedRobotPose = new Pose(estimatedRobotPose2D.getX(DistanceUnit.INCH), estimatedRobotPose2D.getY(DistanceUnit.INCH), estimatedRobotPose2D.getHeading(AngleUnit.RADIANS));
        distanceFromGoal = estimatedRobotPose.distanceFrom(goalPose);
    }

    /**Calculates the pitch (up and down) of the robot hood based on distance to goal*/
    private double servoPitchBasedOnDistance() {
        //TODO: once I have the actual robot I need to add the formula (desmos my goat)
        return 6.7 * distanceFromGoal;
    }

    /**Calculates the yaw (left and right) of the turret based on the distance to goal */
    private double servoYawBasedOnVariables() {
        double ratioGuy = 0.67; //this needs to turn the degrees into servo values (0-1)
        Pose effectiveGoalPose = new Pose(
                goalPose.getX() + pinpoint.getVelX(DistanceUnit.INCH),
                goalPose.getY() + pinpoint.getVelY(DistanceUnit.INCH));
        double dx = effectiveGoalPose.getX() - estimatedRobotPose.getX();
        double dy = effectiveGoalPose.getY() - estimatedRobotPose.getY();
        double headingRadians = Math.atan2(dy, dx);
        double headingDegrees = Math.toDegrees(headingRadians);


        return ratioGuy * headingDegrees;
    }

    /**Calculates the power to give the shooter motors, but does not assign the motor to run the power */
    private void shooterController() {
        controller.setPID(p, i, d);
        double currentVelocity = primaryShooter.getVelocity();
        pid = controller.calculate(currentVelocity, targetShooterVelocity);
    }

    /**Based on the state machine, runs different methods relevant to the current state*/
    private void executeMethodsBasedOnState() {
        if (robotState == RobotStates.AIMING) {
            turretLeft.setPosition(servoYawBasedOnVariables());
            turretRight.setPosition(servoYawBasedOnVariables() + turretOffset);

            hoodLeft.setPosition(servoPitchBasedOnDistance());
            hoodRight.setPosition(servoPitchBasedOnDistance()); //maybe add a hood offset, but doesnt seem likely

            primaryShooter.setPower(pid);
            secondaryShooter.setPower(pid);

            if (isReadyToShoot(gamepad2.a)) { //
                robotState = RobotStates.READY_TO_SHOOT;
            }
        }
        if (robotState == RobotStates.READY_TO_SHOOT) {
            if(gamepad2.x) {
                shoot();
            }
        }
    }

    /** Called whenever robot is told to outtake artifacts*/
    void shoot() {
        gate.setPosition(1);
    }

    /**Called after outtaking all artifacts */
    void endShooting() {
        gate.setPosition(0);
    }

    /**Drive code for robot relative drive (POV)*/
    private void drivePOV() {
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

    /**Initializes the PID controller values*/
    private void initPID() {
        controller = new PIDController(p, i, d);
        //telemetry
    }

    /**Runs all the initializing methods in the class*/
    private void initialize() {
        initializeHardware();
        initVariables();

        initPID();
    }

    /**Initializes the hardware variables to the actual hardware post-init button pressed*/
    private void initializeHardware() {
        primaryShooter = hardwareMap.get(DcMotorEx.class, "primaryShooter");
        secondaryShooter = hardwareMap.get(DcMotor.class, "secondaryShooter");
        hoodLeft = hardwareMap.get(Servo.class, "hoodLeft");
        hoodRight = hardwareMap.get(Servo.class, "hoodRight");

        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightBack");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        secondaryShooter.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**Initializes variables that depend on the initialization process*/
    private void initVariables() {
        if(isRed) {
            goalPose = new Pose(132.5, 135);
        }
        else {
            goalPose = new Pose(12.5, 135);
        }
    }


    /**Reads all the button mappings used, and runs the methods corresponding to the input*/
    private void interpretInputs() {
        //continuous checks


        //one call checks

        //toggle relocalization
        if(gamepad2.y && !gamepad2.yWasPressed()) {
            if(!(robotState == RobotStates.RELOCALIZING)) {
                robotState = RobotStates.IDLE;
            }
            else {
                robotState = RobotStates.RELOCALIZING;
            }
        }
        //toggle shooting (technically a hybrid between continuos and one call)
        if(queueShooting || (gamepad2.a && !gamepad2.aWasPressed())) {
            if(robotState == RobotStates.IDLE) { //if robot is not doing anything, set the robot to calculate
                robotState = RobotStates.AIMING;
            }
            else {
                if(!(robotState == RobotStates.RELOCALIZING || robotState == RobotStates.SHOOTING)) { //robot should not be doing anything else while doing these (except drive)
                    queueShooting = true;
                }
            }
        }
    }

    /**Adds and runs all of the telemetry*/
    private void telemetryMain() {
        telemetry.addData("pShooterVel      = ", primaryShooter.getVelocity());
        telemetry.addData("targetShooterVel = ", targetShooterVelocity);
        telemetry.addData("pShooterPower    = ", primaryShooter.getPower());
        telemetry.addData("hoodPosition     = ", hoodLeft.getPosition());

        telemetry.update();
    }

    /**Called every time the "while(opModeIsActive())" loop updates*/
    private void masterFunction() {
        interpretInputs();
        shooterController();
        updateDistance();
        executeMethodsBasedOnState();
        drivePOV();


        telemetryMain();
    }

    /**Op Mode*/
    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        //run this code

        while (opModeIsActive()) {
            masterFunction();
        }
    }
}
