package org.firstinspires.ftc.teamcode; // make sure this aligns with class location

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

enum ShootingStates {
    IDLE,
    ACCELERATING,
    AT_SPEED,
    SHOOTING
}

@Autonomous(name = "Dauto (Decode Auto)", group = "Examples")
public class Dauto extends OpMode {

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

    private int pathState;

    private final Pose startPose = new Pose(119, 130, Math.toRadians(45)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(89, 99, Math.toRadians(45)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    double targetSpeed = BasicTeleOp.targetSpeed;
    double gateOpen = 0.2;
    double gateClosed = 0.4; //get from basic tele, was letting me nab em for some reason

    private Path scorePreloadRed;
    //private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreloadRed = new Path(new BezierLine(startPose, scorePose));
        scorePreloadRed.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());



    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line.
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();


        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();


        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();


        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();


        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();


        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

         */
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreloadRed);
                setPathState(1);
                break;
            case 1:
                if(follower.isBusy()) {
                    spinFlywheel();
                }
                if(!follower.isBusy()) {
                    shoot();
                    if(canProceed) {
                        setPathState(2);
                        //throw new RuntimeException("idsgoinkla");
                    }
                    break;
                    //throw new RuntimeException("squinkaling");
                }



            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    public void spinFlywheel() {
        double currentVelocity = primaryShooter.getVelocity();
        shooterController.setPID(sP, sI, sD);
        double shooterPid = shooterController.calculate(currentVelocity, targetSpeed);

        setShooterPower(shooterPid);
    }

    public void shoot() {
        canProceed = false;
        //shooterTimer.resetTimer();

        while(!canProceed) {
            boolean completedShooting = false;
            spinFlywheel();

            gate.setPosition(gateOpen);

            if(shooterAtSpeed()) {
                try {
                    intake.setPower(1);
                    sleep(200);
                    intake.setPower(0);
                    sleep(600);
                    intake.setPower(1);
                    sleep(200);
                    intake.setPower(0);
                    sleep(600);
                    intake.setPower(1);
                    sleep(200);
                    intake.setPower(0);
                    sleep(600);
                    intake.setPower(1);
                    sleep(200);
                    intake.setPower(0);
                    sleep(600);
                    intake.setPower(1);
                    sleep(200);
                    intake.setPower(0);
                    sleep(600);
                    intake.setPower(1);
                    sleep(200);
                    intake.setPower(0);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }

            if(pathTimer.getElapsedTime() >= 6000) {
                completedShooting = true;
            }

            if(completedShooting) {
                setShooterPower(0);
                gate.setPosition(gateClosed);
                canProceed = true;
            }
        }
    }

    private void setShooterPower(double value) {
        primaryShooter.setPower(value);
        secondaryShooter.setPower(value);
    }

    public boolean shooterAtSpeed() {
        if(primaryShooter.getVelocity() > 1150 && primaryShooter.getVelocity() < 1220) {
            return true;
        }
        else {
            return false;
        }
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

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
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