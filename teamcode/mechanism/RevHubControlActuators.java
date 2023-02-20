package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RevHubControlActuators {
    private DcMotor intake;
    private DcMotor duckWheel;

    public void init(HardwareMap hwMap){
        intake = (DcMotor) hwMap.get("intake");
        duckWheel = (DcMotor) hwMap.get("wheel");
        duckWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        duckWheel.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setIntakeSpeed(double speed){
        intake.setPower(speed);
    }
    public void setDuckWheelSpeed(double speed){
        duckWheel.setPower(speed);
    }
}

