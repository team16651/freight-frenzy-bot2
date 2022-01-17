package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    private static final double MAX_ROTATION_SPEED = 1.0;
    private DcMotor armMotor = null;
    private Servo handServo = null;
    private DigitalChannel armTouchSensor = null;

    public static final int GROUND = 0;
    public static final int PARK_POSITION = 100;
    public static final int LOW_POSITION = 370;
    public static final int MID_POSITION = 840;
    public static final int HIGH_POSITION = 1270;

    private static final double AUTONOMOUS_POWER = 0.25;

    public Arm(DcMotor armMotor, Servo handServo){
        this.armMotor = armMotor;
        this.handServo = handServo;

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public Arm(DcMotor armMotor, Servo handServo, DigitalChannel armTouchSensor){
        this.armMotor = armMotor;
        this.handServo = handServo;
        this.armTouchSensor = armTouchSensor;

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void grab(){
        handServo.setDirection(Servo.Direction.FORWARD);
        handServo.setPosition(0.0);
    }

    public void release(){
        handServo.setDirection(Servo.Direction.REVERSE);
        handServo.setPosition(0.1);
    }

    public void move (int targetPosition){
        armMotor.setTargetPosition(targetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(AUTONOMOUS_POWER);

        while (armMotor.isBusy()){
            //wait for arm motor to move to position
        }
    }

    public void rotate(double power, boolean up){
        if (up) {
            armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            armMotor.setPower(power * MAX_ROTATION_SPEED);
        }
        else if (armTouchSensor.getState()){
            armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            armMotor.setPower(power * MAX_ROTATION_SPEED);
        }
        else {
            armMotor.setPower(0);
        }
    }
}