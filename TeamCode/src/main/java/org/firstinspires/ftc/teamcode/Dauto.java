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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teleopClasses.Intake;
import org.firstinspires.ftc.teamcode.teleopClasses.Kinematics;
import org.firstinspires.ftc.teamcode.teleopClasses.Shooter;
import org.firstinspires.ftc.teamcode.teleopClasses.Turret;

import java.util.Map;

enum ShootingStates {
    IDLE,
    ACCELERATING,
    AT_SPEED,
    SHOOTING
}

@Autonomous(name = "Rodger the auto (Red)", group = "Examples")
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
            return new Pose(134.5, 140);
        }
        else {
            return new Pose(9.5, 140);
        }
    }

    private boolean canProceed;

    private int pathState;

    private double intakeDriveSpeed = 0.5, normalDriveSpeed = 1;

    private final Pose startPose = new Pose(119, 130, Math.toRadians(222)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(93, 100, Math.toRadians(222));
    private final Pose beforePickupStack1 = new Pose(81, 85, toR(0));
    private final Pose stack1 = new Pose(119,85,toR(0));
    private final Pose beforePickupStack2 = new Pose(86, 60, toR(0)); //ee
    private final Pose stack2 = new Pose(119,60,toR(0)); //ee
    private final Pose parkPose = new Pose(108,78, toR(0));


    ElapsedTime shootTimer = new ElapsedTime();
    int shootStage = 0;

    private Path scorePreload;
    private PathChain moveToBeforeStack1, pickupStack1, score2ndLoad, moveToBeforeStack2, pickupStack2, score3rdLoad, park;

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
                .setLinearHeadingInterpolation(Math.toRadians(222), Math.toRadians(0))
                .build();

        pickupStack1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(beforePickupStack1, stack1)
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        score2ndLoad = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(stack1, scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(222))
                .build();

        moveToBeforeStack2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(scorePose, beforePickupStack2)
                )
                .setLinearHeadingInterpolation(Math.toRadians(222), Math.toRadians(0))
                .build();

        pickupStack2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(beforePickupStack2, stack2)
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        score3rdLoad = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(stack2, scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(222))
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
                follower.followPath(scorePreload);
                shooter.accelerateShooterPID();
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    shoot();
                    if(canProceed) {
                        follower.followPath(moveToBeforeStack1);
                        setPathState(2);
                    }
                    break;
                }
            case 2:
                if(!follower.isBusy() /*follower.atPose(beforePickupStack1, 8, 3, toR(20))*/) {
                    intake.setIntakePower(1);
                    follower.setMaxPower(intakeDriveSpeed);
                    follower.followPath(pickupStack1);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.setMaxPower(normalDriveSpeed);
                    follower.followPath(score2ndLoad);
                    intake.setIntakePower(0);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    shoot();
                    if(canProceed) {
                        follower.followPath(moveToBeforeStack2);
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    intake.setIntakePower(1);
                    follower.setMaxPower(intakeDriveSpeed);
                    follower.followPath(pickupStack2);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.setMaxPower(normalDriveSpeed);
                    follower.followPath(score3rdLoad);
                    intake.setIntakePower(0);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    shoot();
                    shooter.setShooterStopped(false);
                    if(canProceed) {
                        setPathState(8);
                    }
                }
                break;
            case 8:
                shooter.setShooterStopped(true);
                follower.followPath(park);
                setPathState(-1);
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
                if (shootTimer.milliseconds() >= 3000) {
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

        turret.setHomeOverride(true);

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
        turret = new Turret(hardwareMap, kinematics, gamepad1, gamepad2); //3
        shooter = new Shooter(hardwareMap, gamepad1, turret); //4
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