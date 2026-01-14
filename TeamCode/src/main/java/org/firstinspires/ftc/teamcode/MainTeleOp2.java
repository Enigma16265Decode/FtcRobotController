package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teleopClasses.Drive;
import org.firstinspires.ftc.teamcode.teleopClasses.Intake;
import org.firstinspires.ftc.teamcode.teleopClasses.Kinematics;
import org.firstinspires.ftc.teamcode.teleopClasses.Shooter;
import org.firstinspires.ftc.teamcode.teleopClasses.Turret;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "! SC TeleOP \uD83D\uDFE6")
public class MainTeleOp2 extends OpMode {
    Intake intake;
    Shooter shooter;
    Drive drive;
    Kinematics kinematics;
    Turret turret;

    private Follower follower;
    private static final Pose startingPose = new Pose(105,72.0,Math.toRadians(0.0));
    private Supplier<PathChain> pathChain;
    static TelemetryManager telemetryM;

    private final boolean isRed = false;

    private Pose goalPose() {
        if(isRed) {
            return new Pose(134.5, 140);
        }
        else {
            return new Pose(134.5, 4);
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

        //telemetry.addData("target :", shooter.targetSpeed);
        //telemetry.addData("hood pos: ", shooter.getHoodPos());
        telemetry.addData("power : ", shooter.getShooterPower());
        telemetry.addData("(1200) shooter vel: ", shooter.getShooterVelocity());
        //telemetry.addData("intake power: ", intake.getIntakePower());
        telemetry.addData("goal heading: ", kinematics.getHeadingToGoal());
        //telemetry.addData("robot/goal heading", goalHeadingRadians);
        telemetry.addData("offset: ", turret.getOffset());

        telemetry.update();
    }




    private void masterFunction() {
        drive.fieldCentricDrive();
        drive.poseController(isRed);
        //shooter.hoodControl();
        shooter.gateController();
        shooter.shooterController();
        shooter.turretController();
        intake.intakeController(); //make sure this goes after shooter controller

        telemetry();
    }


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

        drive = new Drive(hardwareMap, gamepad1, gamepad2, follower); //1
        kinematics = new Kinematics(follower, goalPose()); //2
        turret = new Turret(hardwareMap, gamepad1, gamepad2, kinematics, false); //3
        shooter = new Shooter(hardwareMap, gamepad1, turret); //4
        intake = new Intake(hardwareMap, gamepad1, shooter); //5
    }
}
