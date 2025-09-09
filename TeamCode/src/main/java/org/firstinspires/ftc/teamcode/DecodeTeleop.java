package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@TeleOp(name="DecodeTeleop", group="Enigma")
public class DecodeTeleop extends LinearOpMode {
    GoBildaPinpointDriver pinpoint;

    double redGoalX = 0, redGoalY = 0;
    double blueGoalX = 0, BlueGoalY = 0;
    boolean isRed = true;

    Pose2D blueGoalPos = new Pose2D(DistanceUnit.INCH, 1.0 ,1.0,AngleUnit.DEGREES,0);
    Pose2D redGoalPos = new Pose2D(DistanceUnit.INCH, 1.0 ,1.0,AngleUnit.DEGREES,0);
    Servo turret;
    DcMotor shooter;



    public double getDistanceToGoal(double robotPosX, double robotPosY, boolean isRed) {
        Pose2D goal = redGoalPos;

        if (!isRed) {
            goal = blueGoalPos; // You can expand this later for blue goal if needed
        }

        double dx = robotPosX - goal.getX(DistanceUnit.INCH);
        double dy = robotPosY - goal.getY(DistanceUnit.INCH);

        return Math.sqrt(dx * dx + dy * dy);
    }


    public double AimGetAngle(boolean robotVelocity) {
        double angle = 1;

        return angle;
    }

    public double angleToServoPos(double angle) {
        double denominator = 360;
        if(angle == 0) {
            angle = 0.001;
        }

        return angle/denominator;
    }

    public static double getAngleBetweenPoints(double x1, double y1, double x2, double y2) {
        // diff in coords (delta)
        double deltaY = y2 - y1;
        double deltaX = x2 - x1;

        // atan2 to get angle in radians, according to wikipedia
        double angleInRadians = Math.atan2(deltaY, deltaX);

        double angleInDegrees = Math.toDegrees(angleInRadians);

        if (angleInDegrees < 0) {
            angleInDegrees += 360;
        }

        return angleInDegrees;
    }

    public void shoot() {
        aimAtGoal();

        //todo make this work
    }

    public void aimAtGoal() {
        double rPosX = pinpoint.getPosX(DistanceUnit.INCH);
        double rPosY = pinpoint.getPosY(DistanceUnit.INCH);
        if(isRed) {
            SmartServo.setSmartPos(hardwareMap, "turret", angleToServoPos(getAngleBetweenPoints(rPosX, rPosY, redGoalPos.getX(DistanceUnit.INCH), redGoalPos.getY(DistanceUnit.INCH))));
        }
        if(!isRed) {
            SmartServo.setSmartPos(hardwareMap, "turret", angleToServoPos(getAngleBetweenPoints(rPosX, rPosY, blueGoalPos.getX(DistanceUnit.INCH), blueGoalPos.getY(DistanceUnit.INCH))));
        }
    }

    private void mainLoop() {
        //run functions
    }
    void initialize() {
        //hardware
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        turret = hardwareMap.get(Servo.class,"turret");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
    }

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        while (opModeIsActive()) {
            mainLoop();
        }
    }

}
