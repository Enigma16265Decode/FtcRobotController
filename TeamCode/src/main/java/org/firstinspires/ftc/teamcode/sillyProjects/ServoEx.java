package org.firstinspires.ftc.teamcode.sillyProjects;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoEx {
    private HardwareMap hardwareMap;
    private Servo servo;
    private Servo complementaryServo;
    private double maxPosition = 1.0;
    private double minPosition = 0.0;
    private boolean hasComplementary = false;
    private double complementaryServoOffset = 0.0;

    public ServoEx(@NonNull HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void setDirection(Servo.Direction direction) {
        servo.setDirection(direction);
    }

    public void setMaxPosition(double max) {
        maxPosition = max;
    }
    public void setMinPosition(double min) {
        minPosition = min;
    }

    public void setPosition(double position) {
        if(position < minPosition) {
            position = minPosition;
        }
        if(position > maxPosition) {
            position = maxPosition;
        }
        servo.setPosition(position);
        if(hasComplementary) {
            complementaryServo.setPosition(position);
        }
    }

    public void setAbsolutePosition(double position) {
        servo.setPosition(position);
        if(hasComplementary) {
            complementaryServo.setPosition(position);
        }
    }
    public void initializeServoHardwareMap(@NonNull String deviceName) {
        servo = hardwareMap.get(Servo.class, deviceName);
    }
    public void initializeServoHardwareMap(@NonNull String deviceName, boolean isComplementaryServo) {
        if(!isComplementaryServo) {
            initializeServoHardwareMap(deviceName);
        }
        else {
            complementaryServo = hardwareMap.get(Servo.class, deviceName);
        }
    }
    public void setComplementaryServo(Servo complementaryServoToSet) {
        complementaryServo = complementaryServoToSet;
    }
    public void initializeComplementaryServoParams(Servo.Direction direction) {
        complementaryServo.setDirection(direction);
    }

    public void initializeComplementaryServoParams(Servo.Direction direction, double offset) {
        complementaryServo.setDirection(direction);
        complementaryServoOffset = offset;
    }
}
