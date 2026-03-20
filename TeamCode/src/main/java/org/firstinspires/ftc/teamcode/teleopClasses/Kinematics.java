package org.firstinspires.ftc.teamcode.teleopClasses;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

public class Kinematics { //this does a lot of the calculations/logic
    Follower follower;
    Pose goalPose;

    public Kinematics(Follower follower, Pose goalPose) {
        this.follower = follower;
        this.goalPose = goalPose;
    }


    public double getHeadingToGoalWithSpeed() {
        Pose effectiveGoalPose = new Pose(
                goalPose.getX(), // + follower.getVelocity().getXComponent(),
                goalPose.getY());// + follower.getVelocity().getYComponent());
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
                goalPose.getX() /*+ follower.getVelocity().getXComponent()*/,
                goalPose.getY() /*+ follower.getVelocity().getYComponent()*/);
        double dx = effectiveGoalPose.getX() - follower.getPose().getX();
        double dy = effectiveGoalPose.getY() - follower.getPose().getY();
        double goalHeadingRadians = Math.atan2(dy, dx);
        double robotHeadingRadians = follower.getHeading(); //this is in radians
        double turretHeadingRadians = goalHeadingRadians - (robotHeadingRadians - Math.PI);
        double turretHeadingDegrees = Math.toDegrees(turretHeadingRadians);

        double unwrappedDegrees = turretHeadingDegrees;

        if(unwrappedDegrees > 270.0) { //changing this to 270 from 180
            unwrappedDegrees -= 360.0;
        }
        if(unwrappedDegrees < -270.0) {
            unwrappedDegrees += 360.0;
        }

        return unwrappedDegrees;
    }

    public double getDistanceToGoal() {
        double x1 = follower.getPose().getX();
        double y1 = follower.getPose().getY();
        double x2 = goalPose.getX();
        double y2 = goalPose.getY();

        double distance = Math.hypot(x1-x2, y1-y2);
        return distance;
    }
}
