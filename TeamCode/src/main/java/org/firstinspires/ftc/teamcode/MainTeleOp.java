package org.firstinspires.ftc.teamcode;

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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teleopClasses.Drive;
import org.firstinspires.ftc.teamcode.teleopClasses.Intake;
import org.firstinspires.ftc.teamcode.teleopClasses.Kinematics;
import org.firstinspires.ftc.teamcode.teleopClasses.Shooter;
import org.firstinspires.ftc.teamcode.teleopClasses.Turret;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "! SC TeleOP \uD83D\uDFE5")
public class MainTeleOp extends OpMode {

    Intake intake;
    Shooter shooter;
    Drive drive;
    Kinematics kinematics;
    Turret turret;

    private Follower follower;
    public static Pose startingPose = new Pose(14,14,Math.toRadians(0));
    private Supplier<PathChain> pathChain;
    static TelemetryManager telemetryM;

    private final boolean isRed = true;

    private Pose goalPose() {
        if(isRed) {
            return new Pose(131.5, 136.5);
        }
        else {
            return null;
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
        telemetry.addData("target :", shooter.targetSpeed);
        telemetry.addData("hood pos: ", shooter.getHoodPos());
        telemetry.addData("power : ", shooter.getShooterPower()); //todo figure out this madness
        telemetry.addData("shooter vel: ", shooter.getShooterVelocity());
        telemetry.addData("intake power: ", intake.getIntakePower());
        telemetry.addData("goal heading: ", kinematics.getHeadingToGoal());

        telemetry.update();
    }




    private void masterFunction() {
        drive.fieldCentricDrive();
        shooter.hoodControl();
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
        shooter = new Shooter(hardwareMap, gamepad1, turret);
        drive = new Drive(hardwareMap, gamepad1, follower);
        kinematics = new Kinematics(shooter, follower, goalPose());
        turret = new Turret(hardwareMap, kinematics);
        intake = new Intake(hardwareMap, gamepad1, kinematics);

        initialize();
    }
}
