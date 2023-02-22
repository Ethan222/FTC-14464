package org.firstinspires.ftc.teamcode.mechanism;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class RevControlHub {
    private DcMotor armMotor;
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

    public void armRuntoPosition(int position, double power){
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(-position);
        while(getArmMotorRotations() < position) {
            armMotor.setPower(power);
        }
        armMotor.setPower(0);
        // armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void armRuntoPosition(int position) {
        armRuntoPosition(position, 1);
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

    public void open() {
        clampServo1.setPosition(.5);
        clampServo2.setPosition(.4);
    }

    public void close() {
        clampServo1.setPosition(.35);
        clampServo2.setPosition(.45);
    }
}
