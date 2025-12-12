package org.firstinspires.ftc.teamcode; // make sure this aligns with class location

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "Billy the auto (Blue)", group = "Examples")
public class Dauto2 extends OpMode {

    ShootingStates currentShootingState = ShootingStates.IDLE;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, shooterTimer;

    public static double sP = BasicTeleOp.sP, sI = BasicTeleOp.sI, sD = BasicTeleOp.sD;
    PIDController shooterController;
    private DcMotorEx primaryShooter;
    private DcMotor secondaryShooter;
    private DcMotor intake;
    private Servo gate;

    private boolean canProceed;
    private boolean isShooting = false;

    private int pathState;

    private final Pose startPose = new Pose(25, 130, Math.toRadians(144)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(53, 95, Math.toRadians(144));
    private final Pose beforePickupStack1 = new Pose(61, 83, toR(180));
    private final Pose stack1 = new Pose(31,83,toR(180));
    private final Pose beforePickupStack2 = new Pose(61, 60, toR(180));
    private final Pose stack2 = new Pose(31,60,toR(180));
    private final Pose parkPose = new Pose(33.5,78);


    double targetSpeed = 1200;
    double gateOpen = 0.2;
    double gateClosed = 0.4; //get from basic tele, was letting me nab em for some reason

    ElapsedTime shootTimer = new ElapsedTime();
    int shootStage = 0;

    private Path scorePreload;
    private PathChain moveToBeforeStack1, pickupStack1, score2ndLoad, moveToBeforeStack2, pickupStack2, score3rdLoad, park;

    private double toR(double toRadian) { //i may get called lazy for this but I dont care :)
        return Math.toRadians(toRadian);
    }

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        moveToBeforeStack1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(scorePose, beforePickupStack1)
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .build();

        pickupStack1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(beforePickupStack1, stack1)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        score2ndLoad = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(stack1, scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                .build();

        moveToBeforeStack2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(scorePose, beforePickupStack2)
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .build();

        pickupStack2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(beforePickupStack2, stack2)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        score3rdLoad = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(stack2, scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                .build();



        park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(scorePose, parkPose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .build();
    }
    /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(follower.isBusy()) {
                    //spinFlywheel();
                }
                if(!follower.isBusy()) {
                    shoot();
                    if(canProceed) {
                        isShooting = false;
                        stopFlywheel();
                        follower.followPath(moveToBeforeStack1);
                        gate.setPosition(gateClosed);
                        setPathState(2);
                    }
                    break;
                }
            case 2:
                if(!follower.isBusy() /*follower.atPose(beforePickupStack1, 8, 3, toR(20))*/) {
                    intake.setPower(1);
                    follower.followPath(pickupStack1);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    //sleepRobot(300);
                    follower.followPath(score2ndLoad);
                    intake.setPower(0);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    shoot();
                    if(canProceed) {
                        follower.followPath(moveToBeforeStack2);
                        gate.setPosition(gateClosed);
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    intake.setPower(1);
                    follower.followPath(pickupStack2);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(score3rdLoad);
                    intake.setPower(0);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    shoot();
                    if(canProceed) {
                        gate.setPosition(gateClosed);
                        setPathState(8);
                    }
                }
                break;
            case 8:
                isShooting = false;
                stopFlywheel();
                follower.followPath(park);
                setPathState(-1);
        }
    }





    public void shoot() {
        canProceed = false;
        isShooting = true;

        gate.setPosition(gateOpen);

        shooterController();

        // Start shooting sequence once flywheel is ready
        if (shootStage == 0 && shooterAtSpeed()) {
            shootTimer.reset();
            shootStage = 1;
        }

        // Stage machine for timed intake pulses
        switch (shootStage) {
            case 1:
                intake.setPower(1);
                if (shootTimer.milliseconds() >= 200) {
                    intake.setPower(0);
                    shootTimer.reset();
                    shootStage = 2;
                }
                break;

            case 2:
                if (shootTimer.milliseconds() >= 600) {
                    intake.setPower(1);
                    shootTimer.reset();
                    shootStage = 3;
                }
                break;

            case 3:
                if (shootTimer.milliseconds() >= 200) {
                    intake.setPower(0);
                    shootTimer.reset();
                    shootStage = 4;
                }
                break;

            case 4:
                if (shootTimer.milliseconds() >= 600) {
                    intake.setPower(1);
                    shootTimer.reset();
                    shootStage = 5;
                }
                break;

            case 5:
                if (shootTimer.milliseconds() >= 200) {
                    intake.setPower(0);
                    shootTimer.reset();
                    shootStage = 6;
                }
                break;

            case 6:
                if (shootTimer.milliseconds() >= 600) {
                    intake.setPower(1);
                    shootTimer.reset();
                    shootStage = 7;
                }
                break;

            case 7:
                if (shootTimer.milliseconds() >= 200) {
                    intake.setPower(0);
                    shootTimer.reset();
                    shootStage = 8;
                }
                break;

            case 8:
                // Shooting finished
                isShooting = false;
                gate.setPosition(gateClosed);
                canProceed = true;
                shootStage = 0;  // reset for next time
                break;
        }
    }


    private void setShooterPower(double value) {
        primaryShooter.setPower(value);
        secondaryShooter.setPower(value);
    }

    public boolean shooterAtSpeed() {
        return primaryShooter.getVelocity() > 1150 && primaryShooter.getVelocity() < 1220;
    }

    private void shooterController() {
        if(isShooting) {
            double currentVelocity = primaryShooter.getVelocity();
            shooterController.setPID(sP, sI, sD);
            double shooterPid = shooterController.calculate(currentVelocity, targetSpeed);

            setShooterPower(shooterPid);
        }
        else {
            setShooterPower(0);
        }
    }

    private void stopFlywheel() {
        primaryShooter.setPower(0);
        secondaryShooter.setPower(0);
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        //shooterController();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("shooter velocity", primaryShooter.getVelocity());
        telemetry.addData("shooter power", primaryShooter.getPower());
        telemetry.addData("shooter is at speed", shooterAtSpeed());
        telemetry.update();
    }

    public void initHardware() {
        shooterController = new PIDController(sP, sI, sD);

        //pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        //maybe do something with telemetry I really don't know im way underqualified

        primaryShooter = hardwareMap.get(DcMotorEx.class, "leftShooter"); //change depending on side
        secondaryShooter = hardwareMap.get(DcMotor.class, "rightShooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        gate = hardwareMap.get(Servo.class, "gate");


        primaryShooter.setDirection(DcMotorSimple.Direction.REVERSE); //again, same as above ^
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        initHardware();

        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.7);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}