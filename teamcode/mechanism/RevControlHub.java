package org.firstinspires.ftc.teamcode.mechanism;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class RevControlHub {
    public DcMotor armMotor;
    private Servo clampServo1;
    private Servo clampServo2;
    private double ticksPerRotation;

    public void init (HardwareMap hwMap){
        armMotor = (DcMotor) hwMap.get("armEx");
        clampServo1 = (Servo) hwMap.get("grab1");
        clampServo2 = (Servo) hwMap.get("grab2");
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setArmMotorSpeed(double speed) {
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setPower(speed);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double getArmMotorRotations() {
        return armMotor.getCurrentPosition();
    }

    public void armRuntoPositionPositive(int position, double power){
       // armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(position);
        while(getArmMotorRotations() < position) {
            armMotor.setPower(-power);
        }
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setPower(0);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void armRuntoPositionPositive(int position, double power, LinearOpMode opmode){
        // armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(position);
        while(opmode.opModeIsActive() && !opmode.isStopRequested() && getArmMotorRotations() < position) {
            armMotor.setPower(-power);
        }
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setPower(0);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void armRuntoPositionNegative(int position, double power){
        // armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(position);
        while(getArmMotorRotations() > position) {
            armMotor.setPower(power);
        }
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setPower(0);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void armRuntoPositionNegative(int position, double power, LinearOpMode opmode){
        // armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(position);
        while(opmode.opModeIsActive() && !opmode.isStopRequested() && getArmMotorRotations() > position) {
            armMotor.setPower(power);
        }
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setPower(0);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void armRuntoPosition(int position, double power) {
        if ((int)getArmMotorRotations() <= position) {
            armRuntoPositionPositive(position, power);
        }
        else {
            armRuntoPositionNegative(position, power);
        }
    }

    public void armRuntoPosition(int position, double power, LinearOpMode opmode) {
        if ((int)getArmMotorRotations() <= position) {
            armRuntoPositionPositive(position, power, opmode);
        }
        else {
            armRuntoPositionNegative(position, power, opmode);
        }
    }
    public void armRuntoPosition(int position) {
        armRuntoPosition(position, 1);
    }

    public void armRuntoPositionPositive(int position) {
        armRuntoPositionPositive(position, 1);
    }

    public void armRuntoPositionNegative(int position) {
        armRuntoPositionNegative(position, 1);
    }

    public void armRuntoPosition(int position, LinearOpMode opmode) {
        armRuntoPosition(position, 1, opmode);
    }

    public void armRuntoPositionPositive(int position, LinearOpMode opmode) {
        armRuntoPositionPositive(position, 1, opmode);
    }

    public void armRuntoPositionNegative(int position, LinearOpMode opmode) {
        armRuntoPositionNegative(position, 1, opmode);
    }

    public void armUp() {
        setArmMotorSpeed(-1);
    }

    public void armDown() {
        setArmMotorSpeed(1);
    }

    public void stopArm() {
        setArmMotorSpeed(0);
    }

    public void setServo1Position (double postion){
        clampServo1.setPosition(postion);
    }

    public void setServo2Position (double postion){
        clampServo2.setPosition(postion);
    }

    public void close() {
        clampServo1.setPosition(.55);
        clampServo2.setPosition(.35);
    }

    public void open() {
        clampServo1.setPosition(.45);
        clampServo2.setPosition(.45);
    }
}
