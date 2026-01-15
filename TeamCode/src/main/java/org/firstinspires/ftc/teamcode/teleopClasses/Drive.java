package org.firstinspires.ftc.teamcode.teleopClasses;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drive {
    private HardwareMap hardwareMap;
    private Gamepad gamepad1;
    private Gamepad gamepad2;

    private Follower follower;

    private DcMotor rightFront, rightRear, leftRear, leftFront;

    public Drive(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Follower follower) {
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");

        //primaryShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //secondaryShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.follower = follower;
    }

    public void fieldCentricDrive() {
        boolean automatedDrive = false;
        double slowmodeMultiplier = 0.3;

        if(!follower.isBusy()) {
            automatedDrive = false;
        }

        //Call this once per loop
        follower.update();

        if (!automatedDrive) {


            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!gamepad1.left_bumper) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x*0.7,
                    false // Robot Centric
            );
                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowmodeMultiplier,
                    -gamepad1.left_stick_x * slowmodeMultiplier,
                    -gamepad1.right_stick_x * slowmodeMultiplier,
                    false // Robot Centric
            );
        }



        //telemetryM.debug("position", follower.getPose());
        //telemetryM.debug("velocity", follower.getVelocity());
        //telemetryM.debug("automatedDrive", automatedDrive);
    }

    public void resetPose(boolean red) {
        if(red) {
            follower.setPose(new Pose(8, 8, Math.toRadians(0)));
        }
        else {
            follower.setPose(new Pose(8, 136, Math.toRadians(0)));
            //follower.setPose(new Pose(136, 8, Math.toRadians(180)));
        }
    }

    public void poseController(boolean red) {
        if(gamepad1.yWasPressed() || gamepad2.yWasPressed()) {
            resetPose(red);
        }
    }

    public void drivePOV() {
        double driveSpeed = 1;
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

    /*
    private void faceGoal() {
        Pose poseToRotateTo = new Pose(follower.getPose().getX()+3, follower.getPose().getY(), Math.toRadians(kinematics.getHeadingToGoal()));

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, poseToRotateTo)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
        follower.followPath(pathChain.get());
    }



    private void faceGoalControl() {
        if(gamepad1.yWasPressed()) {
            faceGoal();
            isRealigning = true;
        }
        if(!follower.isBusy()) {
            isRealigning = false;
        }
    }

     */
}
