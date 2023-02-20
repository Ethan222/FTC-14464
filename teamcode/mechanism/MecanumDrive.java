package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrive {

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    public void init(HardwareMap hwMap){
        frontLeftMotor = (DcMotor) hwMap.get("leftFront");
        frontRightMotor = (DcMotor) hwMap.get("rightFront");
        backLeftMotor = (DcMotor) hwMap.get("leftRear");
        backRightMotor = (DcMotor) hwMap.get("rightRear");
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void DriveRobot(double y, double x, double rx){
        double yy = -y;

        double denominator = Math.max(Math.abs(yy) + Math.abs(x) + Math.abs(rx) , 1 );
        double frontLeftPower = (yy + x + rx) / denominator;
        double backLeftPower = (yy - x + rx) / denominator;
        double frontRightPower = (yy - x - rx) / denominator;
        double backRightPower = (yy + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

    }

    public double getFLMotorRotations() {
        return frontLeftMotor.getCurrentPosition();
    }
    public double getFRMotorRotations() {
        return frontRightMotor.getCurrentPosition();
    }
    public double getBLMotorRotations() {
        return backLeftMotor.getCurrentPosition();
    }
    public double getBRMotorRotations() {
        return backRightMotor.getCurrentPosition();
    }

}
