package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Disabled
@Config
@Autonomous(group = "drive")
public class RightB extends LinearOpMode {
    public static final double D = 17;

    @Override
    public void runOpMode() throws InterruptedException {
        // color sensor
        int zone = 2;   // 1, 2, or 3

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(24 / D)
                .forward(48 / D)
                .build();
        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.PI / 2)  // this should turn right, idk if it does
                .forward(48 / D)
                .build();
        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.PI)
                .forward(24 / D)
                .build();
        // last trajSeq depends on signal
        TrajectorySequence trajSeq4;
        if(zone == 1)
            trajSeq4 = drive.trajectorySequenceBuilder(startPose).back(24 / D).build();
        else if (zone == 3)
            trajSeq4 = drive.trajectorySequenceBuilder(startPose).forward(24 / D).build();
        else
            trajSeq4 = drive.trajectorySequenceBuilder(startPose).build();  // don't move

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(trajSeq1);
        // place cone

        drive.followTrajectorySequence(trajSeq2);
        // pick up cone

        drive.followTrajectorySequence(trajSeq3);
        // place cone

        drive.followTrajectorySequence(trajSeq4);

        while (!isStopRequested() && opModeIsActive());
    }
}
