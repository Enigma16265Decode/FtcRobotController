package org.firstinspires.ftc.teamcode.auto; // make sure this aligns with class location

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.enums.Alliances;
import org.firstinspires.ftc.teamcode.enums.IntakeModes;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teleopClasses.Intake;
import org.firstinspires.ftc.teamcode.teleopClasses.Kinematics;
import org.firstinspires.ftc.teamcode.teleopClasses.LimelightSS;
import org.firstinspires.ftc.teamcode.teleopClasses.Shooter;
import org.firstinspires.ftc.teamcode.teleopClasses.Turret;

//@Autonomous(name = "DautoClose", group = "Examples")
public class DautoClose extends OpMode {
    private static Intake intake;
    private static Kinematics kinematics;
    private static Shooter shooter;
    private static Turret turret;
    private static LimelightSS limelight;
    private Follower follower;
    private Alliances alliance;

    public DautoClose(Alliances alliance) {
        this.alliance = alliance;
    }

    private Pose startPose = new Pose(27.5, 126, Math.toRadians(135)); // Start Pose of our robot.
    private Pose scorePose = new Pose(54, 88, Math.toRadians(180));
    private Pose gatePose = new Pose(22.5, 71, Math.toRadians(180));
    private Pose pickup1Pose = new Pose(21.5, 81, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private Pose pickup1Control = new Pose(49.5, 81);
    private Pose gateControl1 = new Pose(32, 75);
    private Pose pickup2Pose = new Pose(18.5, 58, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private Pose pickup2Control = new Pose(56.5, 56);
    private Pose gateControl2 = new Pose(29.5, 72);
    private Pose pickup3Pose = new Pose(16, 32, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private Pose pickup3Control = new Pose(64, 26);
    private Pose gateControl3 = new Pose(32.5, 53.5);
    private Pose gateIntake = new Pose(11, 56.5, Math.toRadians(135));
    private static Pose parkPose = new Pose(40, 75, Math.toRadians(180));


    //private Path scorePreload;
    private PathChain scorePreload, grabPickup1, openGate1, scorePickup1, grabPickup2, openGate2, scorePickup2, gateIntake1, scoreGate1, gateIntake2, scoreGate2, scoreGate3, gateIntake3, park;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        //scorePreload = new Path(new BezierLine(startPose, scorePose));
        //scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();


        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickup1Control, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        openGate1 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1Pose, gateControl1, gatePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), gatePose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, scorePose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickup2Control, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        openGate2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pose, gateControl2, gatePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), gatePose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, scorePose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading())
                .build();


        gateIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, gateIntake))
                .setLinearHeadingInterpolation(scorePose.getHeading(), gateIntake.getHeading())
                .build();

        scoreGate1 = follower.pathBuilder()
                .addPath(new BezierLine(gateIntake, scorePose))
                .setLinearHeadingInterpolation(gateIntake.getHeading(), scorePose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    public static Pose getParkPose(Alliances currentAlliance) {
        if(currentAlliance == Alliances.BLUE) {
            return parkPose;
        }
        if(currentAlliance == Alliances.RED) {
            return parkPose.mirror();
        }
        else {
            return parkPose;
        }
    }

    private Pose goalPose() {
        if(alliance == Alliances.BLUE) {
            return new Pose(11, 137);
        }
        else {
            return new Pose(133, 137);
        }
    }

    public static Command runShooterPID() {
        return Command.build()
                .setExecute(() -> {
                   shooter.runShooterPID();
                });
    }

    public static Command openGate() {
        return Command.build()
                .setStart(() -> {
                   shooter.setGateOpen();
                })
                .setDone(() -> {
                    return true;
                });
    }

    public static Command closeGate() {
        return Command.build()
                .setStart(() -> {
                    shooter.setGateClosed();
                })
                .setDone(() -> {
                    return true;
                });
    }

    public static Command setIntakeMode(IntakeModes intakeMode) {
        return Command.build()
                .setStart(() -> {
                    intake.setIntakeMode(intakeMode);
                })
                .setDone(() -> {
                    return true;
                });
    }

    public static Command stopIntaking() {
        return Command.build()
                .setStart(() -> {
                    intake.setIntakeMode(IntakeModes.IDLE);
                })
                .setDone(() -> {
                    return true;
                });
    }

    public static Command stopIntakingResidual() {
        return sequential(
                waitMs(200),
                stopIntaking()
        );
    }

    public static Command setHoodPos(double pos) {
        return Command.build()
                .setStart(() -> {
                   shooter.setHoodPos(pos);
                })
                .setDone(() -> {
                    return true;
                });
    }

    public static Command setHoodPosForClose() {
        return sequential(
                setHoodPos(0.68)
        );
    }

    public static Command shootAndWait() {
        return sequential(
                openGate(),
                waitMs(700),
                setIntakeMode(IntakeModes.SHOOT_CLOSE),
                waitMs(600),
                setIntakeMode(IntakeModes.IDLE),
                closeGate()
        );
    }

    public static Command runTurretController() {
        return Command.build()
                .setExecute(() -> {
                    shooter.turretController();
                });
    }

    public static Command runIntake() {
        return Command.build()
                .setExecute(() -> {
                    intake.runIntake();
                }
        );
    }

    public Command autoRoutine() {
        return sequential(
                follow(follower, scorePreload),
                shootAndWait(),

                setIntakeMode(IntakeModes.INTAKE),
                follow(follower, grabPickup1),
                stopIntakingResidual(),
                follow(follower, openGate1),
                follow(follower, scorePickup1, true),
                shootAndWait(),

                setIntakeMode(IntakeModes.INTAKE),
                follow(follower, grabPickup2),
                stopIntakingResidual(),
                follow(follower, openGate2),
                follow(follower, scorePickup2, true),
                shootAndWait(),

                setIntakeMode(IntakeModes.INTAKE),
                follow(follower, gateIntake1, true),
                waitMs(300),
                stopIntakingResidual(),
                follow(follower, scoreGate1),
                shootAndWait()
                //follow(follower, park)
        );
    }

    private void initSubsystems() {
        limelight = new LimelightSS(hardwareMap, alliance);
        intake = new Intake(hardwareMap, gamepad1);
        kinematics = new Kinematics(follower, goalPose());
        turret = new Turret(hardwareMap, kinematics, limelight, alliance);
        shooter = new Shooter(hardwareMap, gamepad1, gamepad2, turret, intake, kinematics);
    }

    //todo make sure is updated :thumbsup:
    private void doMirroring() {
        if(alliance == Alliances.RED) {
            startPose = startPose.mirror();
            scorePose = scorePose.mirror();
            gatePose = gatePose.mirror();
            pickup1Pose = pickup1Pose.mirror();
            pickup1Control = pickup1Control.mirror();
            gateControl1 = gateControl1.mirror();
            pickup2Pose = pickup2Pose.mirror();
            pickup2Control = pickup2Control.mirror();
            gateControl2 = gateControl2.mirror();
            pickup3Pose = pickup3Pose.mirror();
            pickup3Control = pickup3Control.mirror();
            gateControl3 = gateControl3.mirror();
            parkPose = parkPose.mirror();
        }
    }

    @Override
    public void init() {
        doMirroring();

        Scheduler.reset();
        follower = Constants.createFollower(hardwareMap);
        initSubsystems();

        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        //schedule(setIntakeMode(IntakeModes.INTAKE));
        schedule(setHoodPosForClose());
        schedule(runIntake());
        schedule(runShooterPID());
        schedule(runTurretController());
        schedule(autoRoutine());
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        // Feedback to Driver Hub for debugging
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void stop() {}
}