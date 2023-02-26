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
//@Disabled
@Config
@Autonomous(group = "drive-backup", name = "Left Simple")
public class LeftSimple extends LinearOpMode {
    /*
    YELLOW  = Parking Left
    CYAN    = Parking Middle
    MAGENTA = Parking Right
     */

    public enum ParkingPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    private static final int ARM_TO_TOP_TIME = 2200;
    private static final int ARM_PARTWAY_UP_TIME = 350; //380;
    private static final int PURPLE_MIN = 200;  // purple ~210-300
    private static final int CYAN_MIN = 145;    // cyan ~188-197
                                                // yellow ~75-150

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        RevControlHub intake = new RevControlHub();
        intake.init(hardwareMap);

        double angle0 = Math.toRadians(29), angle1 = Math.toRadians(67);
        double distance = 14;
        TrajectorySequence trajSeq0 = drive.trajectorySequenceBuilder(startPose)
                .forward(distance)
                .turn(-angle0)   // turn right
                .forward(2)
                .build();
        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)    // move to closest high junction
                .back(2)
                .turn(angle0 - Math.toRadians(3))    // turn left
                .back(distance - 2)
                //.strafeLeft(24) // debug
                .strafeRight(30)
                .forward(25)
                //.turn(-angle1)    // turn right
                .build();
        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(startPose)    // manuever over top of junction
                .forward(8)
                .build();
        TrajectorySequence trajSeq2a = drive.trajectorySequenceBuilder(startPose)
                .back(5.5)
                .turn(angle1 - Math.toRadians(3))   // turn left
                .build();
        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(startPose)    // move to stack of cones
                .forward(18)
                .turn(Math.toRadians(85))  // turn left
                .build();
        TrajectorySequence trajSeq3a = drive.trajectorySequenceBuilder(startPose) // purple
                .forward(37)
                .build();
        TrajectorySequence trajSeq3b = drive.trajectorySequenceBuilder(startPose)   // unused
                .forward(6)
                .build();
        TrajectorySequence trajSeq4 = drive.trajectorySequenceBuilder(startPose)    // turn around - unused
                .back(5)
                .turn(Math.toRadians(-175))
                .build();

        TrajectorySequence trajSeq5 = drive.trajectorySequenceBuilder(startPose).forward(17).build(); // cyan
        TrajectorySequence trajSeq6 = drive.trajectorySequenceBuilder(startPose).forward(5).build();

        // color sensor
        NormalizedColorSensor colorSensor;
        View relativeLayout;
        float gain = 20;
        final float[] hsvValues = new float[3]; // {hue, saturation, value}
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "cs1");
        colorSensor.setGain(gain);
        if(colorSensor instanceof SwitchableLight)
            ((SwitchableLight)colorSensor).enableLight(true);

        int zone = 3;
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

            if(hsvValues[0] > PURPLE_MIN)
                zone = 3;
            else if(hsvValues[0] > CYAN_MIN)
                zone = 2;
            else
                zone = 1;
            telemetry.addData("Zone", zone);

            telemetry.addData("Time", "%.3f", getRuntime());
            telemetry.update();
        }

        //waitForStart();

        if(isStopRequested()) return;

        double startTime = getRuntime();

        intake.close();

        // move forward & color sense
        drive.followTrajectorySequence(trajSeq0);
        sleep(10);
        // color sensor
        do {
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
            telemetry.addData("Time", "%.3f", getRuntime() - startTime);

            if(hsvValues[0] > PURPLE_MIN)
                zone = 3;
            else if(hsvValues[0] > CYAN_MIN)
                zone = 2;
            else
                zone = 1;
            telemetry.addData("Zone", zone);
            telemetry.update();
        } while (opModeIsActive() && ((hsvValues[0] < 10 && getRuntime() - startTime < 12) || (getRuntime() - startTime) < 6));

        // move to closest high junction
        drive.followTrajectorySequence(trajSeq1);
        // DONT place 1st cone
        /*armUp(intake, ARM_TO_TOP_TIME);   // move arm up
        drive.followTrajectorySequence(trajSeq2); // move forward slightly
        intake.open(); // drop cone
        drive.followTrajectorySequence(trajSeq2a); // move back a little, turn left
        intake.close();
        armDown(intake, ARM_TO_TOP_TIME);    // move arm down */
        drive.followTrajectorySequence(trajSeq3);   // move forward and turn left

        // TODO: test this part
        if(zone == 1) {
            // move to stack of cones
            drive.followTrajectorySequence(trajSeq3a);
        } else if(zone == 2) {
            drive.followTrajectorySequence(trajSeq5);
        }

        drive.turn(Math.toRadians(90)); // turn left
        drive.followTrajectorySequence(trajSeq6);   // move forward a little
        //intake.open();

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
