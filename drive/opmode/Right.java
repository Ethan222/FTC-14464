package org.firstinspires.ftc.teamcode.drive.opmode;

import android.graphics.Color;
import android.view.View;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.RevControlHub;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Disabled
@Config
@Autonomous(group = "drive")
public class Right extends LinearOpMode {
    private static final int ARM_TO_TOP_TIME = 2200;
    private static final int ARM_PARTWAY_UP_TIME = 350; //380;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        RevControlHub intake = new RevControlHub();
        intake.init(hardwareMap);

        // color sensor
        NormalizedColorSensor colorSensor;
        View relativeLayout;
        float gain = 5;
        final float[] hsvValues = new float[3]; // {hue, saturation, value}
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "cs1");
        colorSensor.setGain(gain);
        if(colorSensor instanceof SwitchableLight)
            ((SwitchableLight)colorSensor).enableLight(true);

        int zone = 1;
        while(!isStarted()) {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            telemetry.addLine()
                    .addData("red", "%.3f", colors.red)
                    .addData("green", "%.3f", colors.green)
                    .addData("blue", "%.3f", colors.blue);
            telemetry.addLine()
                    .addData("Hue", "%.3f", hsvValues[0])
                    .addData("Saturation", "%.3f", hsvValues[1])
                    .addData("Value", "%.3f", hsvValues[2]);
            telemetry.addData("Alpha", "%.3f", colors.alpha);
            /*  YELLOW: r=.025,g=.027,b=.027, h=180.000,s=.143,v=.027, a=.075
                CYAN:   r=.027,g=.029,b=.029
                PURPLE: r=.027,g=.028,b=.026
             */
            if(hsvValues[0] > 260) // purple ~330
                zone = 3;
            else if(hsvValues[0] > 130) // cyan ~180
                zone = 2;
            else // yellow ~80
                zone = 1;
            telemetry.addData("Zone", zone);
            telemetry.update();
        }

        double angle = Math.toRadians(58), angle2 = Math.toRadians(25);
        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)    // move to closest high junction
                .forward(3)
                .strafeLeft(20)
                .forward(24)
                .turn(angle)    // turn left
                .build();
        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(startPose)
                .forward(7)
                .build();
        TrajectorySequence trajSeq2a = drive.trajectorySequenceBuilder(startPose)
                .back(5)
                .turn(-angle + .1)   // turn right
                .build();
        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(startPose)    // move to stack of cones
                .forward(18)
                .turn(Math.toRadians(-85))  // turn right
                .build();
        TrajectorySequence trajSeq3a = drive.trajectorySequenceBuilder(startPose)
                .forward(35)
                .build();
        TrajectorySequence trajSeq3b = drive.trajectorySequenceBuilder(startPose)
                .forward(6)
                .build();
        TrajectorySequence trajSeq4 = drive.trajectorySequenceBuilder(startPose)    // turn around
                .back(5)
                .turn(Math.toRadians(175))
                .build();
        // last trajSeq depends on signal
        TrajectorySequence trajSeq5;
        if(zone == 2)
            trajSeq5 = drive.trajectorySequenceBuilder(startPose).forward(20).build();
        else if (zone == 1)
            trajSeq5 = drive.trajectorySequenceBuilder(startPose).forward(40).build();
        else
            trajSeq5 = drive.trajectorySequenceBuilder(startPose).back(2).build();

        waitForStart();

        if(isStopRequested()) return;

        intake.close();

        // move to closest high junction
        drive.followTrajectorySequence(trajSeq1);
        // place 1st cone
        armUp(intake, ARM_TO_TOP_TIME);   // move arm up
        drive.followTrajectorySequence(trajSeq2); // move forward slightly
        intake.open(); // drop cone
        drive.followTrajectorySequence(trajSeq2a); // move back a little, turn left
        armDown(intake, ARM_TO_TOP_TIME);    // move arm down
        drive.followTrajectorySequence(trajSeq3);   // move forward and turn right

        // TODO: test this part
        if(zone == 3) {
            // move to stack of cones
            drive.followTrajectorySequence(trajSeq3a);
            // pick up 2nd cone
            armUp(intake, ARM_PARTWAY_UP_TIME); // move arm partway up
            drive.followTrajectorySequence(trajSeq3b);  // move forward slightly
            intake.close();
        } else {
            if(zone == 2) {
                trajSeq5 = drive.trajectorySequenceBuilder(startPose).forward(20).build();
                drive.followTrajectorySequence(trajSeq5);
            }
            drive.turn(Math.toRadians(-90)); // turn right
        }

        //armUp(intake, ARM_TO_TOP_TIME - ARM_PARTWAY_UP_TIME); // move arm up

        while (!isStopRequested() && opModeIsActive());
    }

    public void armUp(RevControlHub intake, int time) {
        intake.setArmMotorSpeed(-1);
        sleep(time);
        intake.setArmMotorSpeed(0);
    }

    public void armDown(RevControlHub intake, int time) {
        intake.setArmMotorSpeed(1);
        sleep(time);
        intake.setArmMotorSpeed(0);
    }
}
