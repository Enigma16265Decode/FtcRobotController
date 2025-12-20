package org.firstinspires.ftc.teamcode.teleopClasses;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

public class Kinematics { //this does a lot of the calculations/logic
    Shooter shooter;
    Follower follower;
    Pose goalPose;

    public Kinematics(Shooter shooter, Follower follower, Pose goalPose) {
        this.shooter = shooter;
        this.follower = follower;
        this.goalPose = goalPose;
    }
    public boolean isShooterAtSpeed() {
        if(shooter.getShooterVelocity() > 1150 && shooter.getShooterVelocity() < 1220) {
            return true;
        }
        else {
            return false;
        }
    }

    public double getHeadingToGoalWithSpeed() {
        Pose effectiveGoalPose = new Pose(
                goalPose.getX() + follower.getVelocity().getXComponent(),
                goalPose.getY() + follower.getVelocity().getYComponent());
        double dx = effectiveGoalPose.getX() - follower.getPose().getX();
        double dy = effectiveGoalPose.getY() - follower.getPose().getY();
        double goalHeadingRadians = Math.atan2(dy, dx);
        double robotHeadingRadians = Math.toRadians(follower.getHeading()); //this is in radians
        double turretHeadingRadians = goalHeadingRadians - robotHeadingRadians;
        double turretHeadingDegrees = Math.toDegrees(turretHeadingRadians);

        double unwrappedDegrees = turretHeadingDegrees;

        if(unwrappedDegrees > 360.0) {
            return (unwrappedDegrees - 360.0);
        }
        else if(unwrappedDegrees < 360.0) {
            return (unwrappedDegrees + 360.0);
        }
        else {
            return unwrappedDegrees;
        }
    }

    public double getHeadingToGoal() {
        Pose effectiveGoalPose = new Pose(
                goalPose.getX() + follower.getVelocity().getXComponent(),
                goalPose.getY() + follower.getVelocity().getYComponent());
        double dx = effectiveGoalPose.getX() - follower.getPose().getX();
        double dy = effectiveGoalPose.getY() - follower.getPose().getY();
        double goalHeadingRadians = Math.atan2(dy, dx);
        double robotHeadingRadians = Math.toRadians(follower.getHeading()); //this is in radians
        double turretHeadingRadians = goalHeadingRadians - robotHeadingRadians;
        double turretHeadingDegrees = Math.toDegrees(turretHeadingRadians);

        double unwrappedDegrees = turretHeadingDegrees;

        if(unwrappedDegrees > 360.0) {
            return (unwrappedDegrees - 360.0);
        }
        else if(unwrappedDegrees < 360.0) {
            return (unwrappedDegrees + 360.0);
        }
        else {
            return unwrappedDegrees;
        }
    }
}
