package org.firstinspires.ftc.teamcode.teleopClasses;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.enums.Alliances;
import org.firstinspires.ftc.teamcode.enums.LimelightStates;

import java.util.List;

public class LimelightSS {
    private HardwareMap hardwareMap;
    private Alliances alliance;
    private Limelight3A limelight;
    private LimelightStates limelightState;
    private final int blueID = 20, redID = 24;
    public LimelightSS(HardwareMap hardwareMap, Alliances alliance) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        this.hardwareMap = hardwareMap;
        this.alliance = alliance;
    }

    public void limelightController() {
        LLResult result = limelight.getLatestResult();
        setLimelightState(result);
    }

    private int getLimelightID(LLResult result) {
        int id = -1;
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
        if(result.isValid() && id != -1 && staleness < 100) {
            if(fiducialIsValidTarget()) {
                double threshold = 2;
                if(tx < -threshold) {
                    limelightState = LimelightStates.OFF_LEFT;
                }
                else if(tx > threshold) {
                    limelightState = LimelightStates.OFF_RIGHT;
                }
                else {
                    limelightState = LimelightStates.CENTERED;
                }
            }
            else {
                limelightState = LimelightStates.LOST;
            }
        }
        else {
            limelightState = LimelightStates.LOST;
        }
    }

    public double getFiducialTx() {
        return limelight.getLatestResult().getTx();
    }
    public boolean fiducialIsValidTarget() {
        LLResult result = limelight.getLatestResult();
        if(
                (alliance == Alliances.RED && getLimelightID(result) == redID) ||
                (alliance == Alliances.BLUE && getLimelightID(result) == blueID)
        ) {
            return true;
        }
        else {
            return false;
        }
    }


    public LimelightStates getLimelightState() {return limelightState;}
}
