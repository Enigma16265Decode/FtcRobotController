package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.enums.Alliances;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teleopClasses.Drive;
import org.firstinspires.ftc.teamcode.teleopClasses.Intake;
import org.firstinspires.ftc.teamcode.teleopClasses.Kinematics;
import org.firstinspires.ftc.teamcode.teleopClasses.LimelightSS;
import org.firstinspires.ftc.teamcode.teleopClasses.Shooter;
import org.firstinspires.ftc.teamcode.teleopClasses.Turret;

import java.util.function.Supplier;

//@Configurable
//@TeleOp(name = "! SC TeleOP \uD83D\uDFE5")
public class MainTeleOp extends OpMode {
    LimelightSS limelight;
    Intake intake;
    Shooter shooter;
    Drive drive;
    Kinematics kinematics;
    Turret turret;

    private Follower follower;
    private Alliances alliance;
    private static final Pose startingPose = new Pose(105,72, Math.toRadians(0));
    private Supplier<PathChain> pathChain;
    static TelemetryManager telemetryM;
    public Pose goalPose() {
        if(alliance == Alliances.RED) {
            return new Pose(135, 141);
        }
        else {
            return new Pose(135, 3);
        }
    }

    public MainTeleOp(Alliances alliance) {
        if(alliance == Alliances.RED) {
            alliance = Alliances.RED;
        }
        else {
            alliance = Alliances.BLUE;
        }
    }


    private void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

    }


    private void telemetry() {
        double dx = goalPose().getX() - follower.getPose().getX();
        double dy = goalPose().getY() - follower.getPose().getY();
        double goalHeadingRadians = Math.atan2(dy, dx);

        /*
        telemetry.addData("Hood Pos: ", shooter.getHoodPos());
        telemetry.addData("Shooter Power : ", shooter.getShooterPower());
        telemetry.addData("Shooter vel: ", shooter.getShooterVelocity());
        telemetry.addData("Shooting Range: ", shooter.getShootingRangeString());
        telemetry.addData("Gate Pos", shooter.getGatePos());
        telemetry.addData("velocity error: ", shooter.getVelocityError());
        //telemetry.addData("presses: ", shooter.getPresses());

         */
        telemetry.addData("targetPos", turret.getTargetPos());
        telemetry.addData("turretPos", turret.getCurrentPos());
        telemetry.addData("deg readout", kinematics.getHeadingToGoal());

        telemetry.update();
    }




    private void runTeleOp() {
        //limelight.limelightController();

        drive.fieldCentricDrive();
        drive.poseController(alliance);
        //shooter.toggleShootingRange();
        //shooter.hoodControl();
        //shooter.shooterController();
        //shooter.turretController();
        //shooter.setRgbBasedOnDistance();
        //shooter.shootControl();
        intake.intakeController();
        turret.setTargetBasedOnHeadingToGoal();
        turret.moveTurret();

        telemetry();
    }

    @Override
    public void init_loop() {
        //telemetry.addData("Current Turret Pos:",turret.getCurrentPos());
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        //shooter.initHood();
    }

    @Override
    public void loop() {
        runTeleOp();
    }

    @Override
    public void init() {
        initialize();

        //limelight = new LimelightSS(hardwareMap, alliance);
        intake = new Intake(hardwareMap, gamepad1);
        drive = new Drive(hardwareMap, gamepad1, gamepad2, follower);
        kinematics = new Kinematics(follower, goalPose());
        turret = new Turret(hardwareMap, kinematics, alliance);
        //shooter = new Shooter(hardwareMap, gamepad1, turret, intake, kinematics);
    }
}
