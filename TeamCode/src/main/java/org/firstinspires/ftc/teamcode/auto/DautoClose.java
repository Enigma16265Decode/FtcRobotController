package org.firstinspires.ftc.teamcode.auto; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.commands.Commands.*;

import org.firstinspires.ftc.teamcode.enums.Alliances;
import org.firstinspires.ftc.teamcode.enums.IntakeModes;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teleopClasses.Intake;

//@Autonomous(name = "DautoClose", group = "Examples")
public class DautoClose extends OpMode {
    private static Intake intake;
    private Follower follower;
    private Alliances alliance;

    public DautoClose(Alliances alliance) {
        this.alliance = alliance;
    }

    private final Pose startPose = new Pose(25.5, 127.5, Math.toRadians(135)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(48, 95.5, Math.toRadians(180));
    private final Pose gatePose = new Pose(21, 71, Math.toRadians(180));
    private final Pose pickup1Pose = new Pose(21.5, 81, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1Control = new Pose(49.5, 81);
    private final Pose gateControl1 = new Pose(32, 75);
    private final Pose pickup2Pose = new Pose(18.5, 58, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Control = new Pose(56.5, 56);
    private final Pose gateControl2 = new Pose(29.5, 72);
    private final Pose pickup3Pose = new Pose(16.5, 34, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Control = new Pose(62, 31);
    private final Pose gateControl3 = new Pose(32.5, 53.5);
    private final Pose parkPose = new Pose(40, 75, Math.toRadians(180));


    //private Path scorePreload;
    private PathChain scorePreload, grabPickup1, openGate1, scorePickup1, grabPickup2, openGate2, scorePickup2, grabPickup3, openGate3, scorePickup3, park;

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

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickup3Control ,pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        openGate3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup3Pose, gateControl3, gatePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), gatePose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
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

    public static Command shootAndWait() {
        return sequential(
                setIntakeMode(IntakeModes.OUTTAKE),
                waitMs(600),
                setIntakeMode(IntakeModes.IDLE)
        );
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
                setIntakeMode(IntakeModes.IDLE),
                follow(follower, openGate1, true),
                follow(follower, scorePickup1, true),
                shootAndWait(),

                setIntakeMode(IntakeModes.INTAKE),
                follow(follower, grabPickup2),
                setIntakeMode(IntakeModes.IDLE),
                follow(follower, openGate2, true),
                follow(follower, scorePickup2, true),
                shootAndWait(),

                setIntakeMode(IntakeModes.INTAKE),
                follow(follower, grabPickup3),
                setIntakeMode(IntakeModes.IDLE),
                follow(follower, openGate3, true),
                follow(follower, scorePickup3, true),
                shootAndWait(),
                follow(follower, park)
        );
    }

    private void initSubsystems() {
        intake = new Intake(hardwareMap, gamepad1);
    }

    private void doMirroring() {
        if(alliance == Alliances.RED) {
            startPose.mirror();
            scorePose.mirror();
            gatePose.mirror();
            pickup1Pose.mirror();
            pickup1Control.mirror();
            gateControl1.mirror();
            pickup2Pose.mirror();
            pickup2Control.mirror();
            gateControl2.mirror();
            pickup3Pose.mirror();
            pickup3Control.mirror();
            gateControl3.mirror();
            parkPose.mirror();
        }
    }

    @Override
    public void init() {
        doMirroring();
        initSubsystems();

        Scheduler.reset();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        //schedule(setIntakeMode(IntakeModes.INTAKE));
        schedule(runIntake());
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