package org.firstinspires.ftc.teamcode; // make sure this aligns with class location

import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teleopClasses.Intake;
import org.firstinspires.ftc.teamcode.teleopClasses.Kinematics;
import org.firstinspires.ftc.teamcode.teleopClasses.Shooter;
import org.firstinspires.ftc.teamcode.teleopClasses.Turret;

enum ShootingStates {
    IDLE,
    ACCELERATING,
    AT_SPEED,
    SHOOTING
}

@Autonomous(name = "Ronald the auto (Red)", group = "Examples")
public class Dauto extends OpMode {
    Intake intake;
    Shooter shooter;
    Kinematics kinematics;
    Turret turret;
    ShootingStates currentShootingState = ShootingStates.IDLE;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, shooterTimer;
    boolean isRed = true;
    private Pose goalPose() {
        if(isRed) {
            return new Pose(138, 142);
        }
        else {
            return new Pose(144, 72);
        }
    }

    private boolean canProceed, canProceedIntake = false;
    private int pathState;
    int holdStage = 0;

    private final double intakeDriveSpeed = 0.8, normalDriveSpeed = 1;

    private final Pose startPose = new Pose(119, 130, Math.toRadians(222)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(91, 98, Math.toRadians(222));
    private final Pose beforePickupStack1 = new Pose(81, 85, toR(0));
    private final Pose stack1 = new Pose(124,85,toR(0));
    private final Pose beforePickupStack2 = new Pose(94, 62, toR(0)); //private final Pose beforePickupStack2 = new Pose(86, 62, toR(0));
    private final Pose pickupStack2Control = new Pose(123, 58);
    private final Pose stack2 = new Pose(127,63,toR(0)); //59
    private final Pose shoot3rdControl = new Pose(90,64);
    private final Pose beforePickupStack3 = new Pose(86, 37, toR(0));
    private final Pose stack3 = new Pose(127,37,toR(0));
    private final Pose parkPose = new Pose(105,72, toR(0));


    ElapsedTime shootTimer = new ElapsedTime();
    ElapsedTime holdTimer = new ElapsedTime();
    int shootStage = 0;

    private Path scorePreload;
    private PathChain moveToBeforeStack1, pickupStack1, score2ndLoad, moveToBeforeStack2, pickupStack2, score3rdLoad, moveToBeforeStack3, pickupStack3, score4thLoad, park;

    private double toR(double toRadian) {
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
                .setLinearHeadingInterpolation(scorePose.getHeading(), beforePickupStack1.getHeading())
                .build();

        pickupStack1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(beforePickupStack1, stack1)
                )
                .setLinearHeadingInterpolation(beforePickupStack1.getHeading(), stack1.getHeading())
                .build();

        score2ndLoad = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(stack1, scorePose)
                )
                .setLinearHeadingInterpolation(stack1.getHeading(), scorePose.getHeading())
                .build();

        moveToBeforeStack2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(scorePose, beforePickupStack2)
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), beforePickupStack2.getHeading())
                .build();
        pickupStack2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(beforePickupStack2, pickupStack2Control, stack2)
                )
                .build();


        /*
        pickupStack2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(beforePickupStack2, stack2)
                )
                .setLinearHeadingInterpolation(beforePickupStack2.getHeading(), stack2.getHeading())
                .build();


        score3rdLoad = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(stack2, scorePose)
                )
                .setLinearHeadingInterpolation(stack2.getHeading(), scorePose.getHeading())
                .build();

         */

        score3rdLoad = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(stack2, shoot3rdControl, scorePose)
                ).setLinearHeadingInterpolation(stack2.getHeading(), scorePose.getHeading())
                .build();

        moveToBeforeStack3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(scorePose, beforePickupStack3)
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), beforePickupStack3.getHeading())
                .build();

        pickupStack3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(beforePickupStack3, stack3)
                )
                .setLinearHeadingInterpolation(beforePickupStack3.getHeading(), stack3.getHeading())
                .build();

        score4thLoad = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(stack3, scorePose)
                )
                .setLinearHeadingInterpolation(stack3.getHeading(), scorePose.getHeading())
                .build();

        park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(scorePose, parkPose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(0))
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
                shooter.setHoodForClose();
                follower.followPath(scorePreload);
                shooter.accelerateShooterPID();
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    shoot();
                    if(canProceed) {
                        follower.followPath(moveToBeforeStack2);
                        setPathState(2);
                    }
                    break;
                }
            case 2:
                if(!follower.isBusy() /*follower.atPose(beforePickupStack1, 8, 3, toR(20))*/) {
                    shooter.accelerateShooterPID();
                    follower.setMaxPower(intakeDriveSpeed);

                    intake.setIntakePower(1);
                    follower.setMaxPower(intakeDriveSpeed-0.2);
                    follower.followPath(pickupStack2);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.setMaxPower(1);
                    shooter.accelerateShooterPID();

                    follower.setMaxPower(normalDriveSpeed);
                    follower.followPath(score3rdLoad);
                    intake.setIntakePower(0);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    shoot();
                    if(canProceed) {
                        follower.followPath(moveToBeforeStack1);
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    shooter.accelerateShooterPID();

                    intake.setIntakePower(1);
                    follower.setMaxPower(intakeDriveSpeed);
                    follower.followPath(pickupStack1);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    shooter.accelerateShooterPID();

                    follower.setMaxPower(normalDriveSpeed);
                    follower.followPath(score2ndLoad);
                    intake.setIntakePower(0);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    shoot();
                    if(canProceed) {
                        follower.followPath(moveToBeforeStack3);
                        setPathState(8);
                    }
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    shooter.accelerateShooterPID();

                    intake.setIntakePower(1);
                    follower.setMaxPower(intakeDriveSpeed);
                    follower.followPath(pickupStack3);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    shooter.accelerateShooterPID();

                    follower.setMaxPower(normalDriveSpeed);
                    follower.followPath(score4thLoad);
                    intake.setIntakePower(0);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    shoot();
                    shooter.setShooterStopped(false);
                    if(canProceed) {
                        setPathState(11);
                    }
                }
                break;
            case 11:
                shooter.setShooterStopped(true);
                follower.followPath(park);
                setPathState(-1);
        }
        shooter.accelerateShooterPID();
    }


    private void holdIntake() {
        canProceedIntake = false;
        intake.setIntakePower(1);
        if(holdStage == 0) {
            holdTimer.reset();
            holdStage = 1;
        }

        switch (holdStage) {
            case 1:
                if(holdTimer.milliseconds() > 22000) {
                    intake.setIntakePower(0);
                    holdStage = 2;
                    break;
                }
            case 2:
                canProceedIntake = true;
                holdStage = 0;
                break;
        }

    }

    public void shoot() {
        canProceed = false;

        shooter.accelerateShooterPID();
        shooter.setGateOpen();

        // Start shooting sequence once flywheel is ready
        if (shootStage == 0 && shooter.isShooterAtSpeed()) {
            intake.setIntakePower(0);
            shootTimer.reset();
            shootStage = 1;
        }

        // Stage machine for timed intake pulses
        switch (shootStage) {
            case 1:
                if(shootTimer.milliseconds() >= 1000) {
                    shootTimer.reset();
                    shootStage = 2;
                }
                break;
            case 2:
                intake.setIntakePower(1);
                if (shootTimer.milliseconds() >= 1500) {
                    intake.setIntakePower(0);
                    shootTimer.reset();
                    // Shooting finished
                    shooter.setGateClosed();
                    canProceed = true;
                    shootStage = 0;  // reset for next time
                    break;
                }
                break;
        }
    }

    public void shootOld() {
        canProceed = false;

        shooter.accelerateShooterPID();
        shooter.setGateOpen();

        // Start shooting sequence once flywheel is ready
        if (shootStage == 0 && shooter.isShooterAtSpeed()) {
            intake.setIntakePower(0);
            shootTimer.reset();
            shootStage = 1;
        }

        // Stage machine for timed intake pulses
        switch (shootStage) {
            case 1:
                if(shootTimer.milliseconds() >= 1000) {
                    shootTimer.reset();
                    shootStage = 2;
                }
                break;
            case 2:
                intake.setIntakePower(1);
                if (shootTimer.milliseconds() >= 200) {
                    intake.setIntakePower(0);
                    shootTimer.reset();
                    shootStage = 3;
                }
                break;

            case 3:
                if (shootTimer.milliseconds() >= 600) {
                    intake.setIntakePower(1);
                    shootTimer.reset();
                    shootStage = 4;
                }
                break;

            case 4:
                if (shootTimer.milliseconds() >= 200) {
                    intake.setIntakePower(0);
                    shootTimer.reset();
                    shootStage = 5;
                }
                break;

            case 5:
                if (shootTimer.milliseconds() >= 600) {
                    intake.setIntakePower(1);
                    shootTimer.reset();
                    shootStage = 6;
                }
                break;

            case 6:
                if (shootTimer.milliseconds() >= 200) {
                    intake.setIntakePower(0);
                    shootTimer.reset();
                    shootStage = 7;
                }
                break;

            case 7:
                if (shootTimer.milliseconds() >= 600) {
                    intake.setIntakePower(1);
                    shootTimer.reset();
                    shootStage = 8;
                }
                break;

            case 8:
                if (shootTimer.milliseconds() >= 200) {
                    intake.setIntakePower(0);
                    shootTimer.reset();
                    shootStage = 9;
                }
                break;

            case 9:
                // Shooting finished
                shooter.setGateClosed();
                canProceed = true;
                shootStage = 0;  // reset for next time
                break;
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
        turret.moveTurret();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("shooter vel: ", shooter.getShooterVelocity());
        telemetry.update();
    }



    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        kinematics = new Kinematics(follower, goalPose()); //2
        turret = new Turret(hardwareMap, gamepad1, gamepad2, kinematics, true); //3
        shooter = new Shooter(hardwareMap, gamepad1, turret, kinematics); //4
        intake = new Intake(hardwareMap, gamepad1, shooter); //5

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

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