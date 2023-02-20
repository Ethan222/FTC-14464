package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanism.RevControlHub;

@Config
@Autonomous
public class ArmTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        RevControlHub intake = new RevControlHub();
        intake.init(hardwareMap);

        waitForStart();

        intake.armRuntoPosition(2500);

        while(opModeIsActive() && !isStopRequested()) {
        }
    }
}
