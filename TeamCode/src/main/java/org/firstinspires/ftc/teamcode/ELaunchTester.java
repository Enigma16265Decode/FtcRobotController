package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.LinkedList;
import java.util.Queue;

@TeleOp(name = "launcher mclauncherface")
public class ELaunchTester extends LinearOpMode {
    public class GradualLossDetector {
        private final Queue<Double> window = new LinkedList<>();
        private final int windowSize;
        private final double tolerance; // Minimum decrease per step to consider it a "loss"

        public GradualLossDetector(int windowSize, double tolerance) {
            this.windowSize = windowSize;
            this.tolerance = tolerance;
        }

        public boolean addValue(double newValue) {
            window.add(newValue);

            if (window.size() > windowSize) {
                window.poll(); // Maintain fixed window size
            }

            return isGraduallyDecreasing();
        }

        private boolean isGraduallyDecreasing() {
            if (window.size() < windowSize) return false;

            Double[] values = window.toArray(new Double[0]);

            for (int i = 1; i < values.length; i++) {
                if ((values[i - 1] - values[i]) < tolerance) {
                    return false; // Not enough decrease or value increased
                }
            }

            return true; // All values in the window are decreasing sufficiently
        }
    }

    GradualLossDetector detector = new GradualLossDetector(200, 100);



    DcMotorEx shooter;
    double[] inputs;

    void initialize() {
        //hardware
        shooter = hardwareMap.get(DcMotorEx.class, "rightRear");

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void exportTelemetry() {
        boolean isLosing = detector.addValue(shooter.getVelocity());

        telemetry.addData("launcher velocity: ", shooter.getVelocity());
        telemetry.addData("detected ball: ", isLosing);
        telemetry.update();
    }
    void appendValueToList() {
        int x = inputs.length + 1;
        inputs[x] = shooter.getVelocity();
    }

    void updatePower() {
        if (gamepad1.x) {
            shooter.setPower(0);
        }
        if (gamepad1.y) {
            shooter.setPower(0.7);
        }
        if (gamepad1.b) {
            shooter.setPower(1);
        }
    }

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        while (opModeIsActive()) {
            updatePower();
            exportTelemetry();


            sleep(1);
        }
    }
}
