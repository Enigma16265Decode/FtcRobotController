package org.firstinspires.ftc.teamcode.limelight;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.LimelightStates;

import java.util.List;

@TeleOp(name = "! limelight testig")
public class LimelightTest extends OpMode {
    LimelightStates limelightState;
    private Limelight3A limelight;
    CRServo servo;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        servo = hardwareMap.get(CRServo.class, "servo");
        servo.setDirection(CRServo.Direction.FORWARD);

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.
         */
        limelight.start();
    }

    private int getLimelightID(LLResult result) {
        int id = 0;
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            id = fiducial.getFiducialId(); // The ID number of the fiducial
        }
        return id;
    }

    private void setLimelightState(LLResult result) {
        int id = getLimelightID(result);

        double tx = result.getTx();
        long staleness = result.getStaleness();
        if(result.isValid() && id != 0 && staleness < 100) {
            double threshold = 4;
            if(tx > threshold) {
                limelightState = LimelightStates.OFF_LEFT;
            }
            if(tx < (threshold*-1)) {
                limelightState = LimelightStates.OFF_RIGHT;
            }
            if(tx < threshold && tx > (threshold*-1)) {
                limelightState = LimelightStates.CENTERED;
            }
        }
        else {
            limelightState = LimelightStates.LOST;
        }
    }

    private void setLimelightStateHuman(LLResult result) {
        long staleness = result.getStaleness();

        //boolean isHuman = result.getFiducialResults().equals("person") //result.getClassName().equals("person");

        if(result != null && result.isValid()){
            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();

            for(Object d : detections){
                telemetry.addData("Class", d.getClass().getName());
                telemetry.addData("Confidence", d.getClass().getName());
            }
        }

        double tx = result.getTx();
        if(result.isValid() && staleness < 100) {
            double threshold = 4;

            if(tx > threshold) {
                limelightState = LimelightStates.OFF_LEFT;
            }
            else if(tx < -threshold) {
                limelightState = LimelightStates.OFF_RIGHT;
            }
            else {
                limelightState = LimelightStates.CENTERED;
            }

        } else {
            limelightState = LimelightStates.LOST;
        }
    }


    private void trackAprilTags(LLResult result) {
        if (result.isValid()) {
            double tx = result.getTx();

            double amountToMove = (tx / 100);
            if (limelightState == LimelightStates.OFF_LEFT) {
                //servo.setPosition(servo.getPosition() + amountToMove);
                servo.setPower(-0.1);
            }
            if (limelightState == LimelightStates.OFF_RIGHT) {
                //servo.setPosition(servo.getPosition() - amountToMove);
                servo.setPower(0.1);
            }
            if (limelightState == LimelightStates.CENTERED || limelightState == LimelightStates.LOST) {
                servo.setPower(0);
            }


            telemetry.addData("tx", result.getTx());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("off", limelightState);
        }
    }

    private void trackHumans(LLResult result) {
        if (result.isValid()) {
            double tx = result.getTx();

            double amountToMove = (tx / 100);
            if (limelightState == LimelightStates.OFF_LEFT) {
                //servo.setPosition(servo.getPosition() + amountToMove);
                servo.setPower(-0.1);
            }
            if (limelightState == LimelightStates.OFF_RIGHT) {
                //servo.setPosition(servo.getPosition() - amountToMove);
                servo.setPower(0.1);
            }
            if (limelightState == LimelightStates.CENTERED || limelightState == LimelightStates.LOST) {
                servo.setPower(0);
            }


            telemetry.addData("tx", result.getTx());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("off", limelightState);
            telemetry.addData("name", result.getClass().getName());
        }
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();
        setLimelightStateHuman(result);

        trackHumans(result);
    }
}