/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.RevControlHub;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Config
@Autonomous
public class Left2 extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    final int[] TAGS  = { 0, 1, 2 };

    AprilTagDetection tagOfInterest = null;
    int zone = 2;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        RevControlHub intake = new RevControlHub();
        intake.init(hardwareMap);

        double startTime = .5;
        final double uptime = .915; //91; //92; //.91; //.94; //.92; //.87; //.99
        TrajectorySequence toLowJunction0 = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    intake.close();
                })
                .addTemporalMarker(startTime, () -> {
                    intake.setArmMotorSpeed(-1);    // arm up
                })
                .addTemporalMarker(startTime + uptime, () -> {
                    intake.setArmMotorSpeed(0);     // stop arm
                })
                .forward(2)
                .strafeLeft(18) //17.7)
                .forward(25.5) //.9) //.7)
                .build();
        startTime = 1;
        //double downTime = .6;
        TrajectorySequence toStack1 = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(12)
                /*.addTemporalMarker(startTime, () -> {
                    intake.setArmMotorSpeed(1); // arm down
                })
                .addTemporalMarker(startTime + downTime, () -> {
                    intake.setArmMotorSpeed(0);
                })*/
                .forward(16) //-?
                .turn(Math.toRadians(90.5)) // turn left
                .forward(3)
                .build();
        startTime = .2;
        double upTime = .4;
        double dist = 18, strafe = 15.3; //14.8; //13.9; //.4;
        TrajectorySequence toJunction1 = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(startTime, () -> {
                    intake.setArmMotorSpeed(-1); // arm up
                })
                .addTemporalMarker(startTime + upTime, () -> {
                    intake.setArmMotorSpeed(0);
                })
                .back(dist)
                .strafeLeft(strafe)
                .build();
        startTime = 1;
        double downTime = .44; //42; //.46;
        TrajectorySequence toStack2 = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(startTime, () -> {
                    intake.setArmMotorSpeed(1); // arm down
                })
                .addTemporalMarker(startTime + downTime, () -> {
                    intake.setArmMotorSpeed(0);
                })
                .strafeRight(strafe - 1)
                .forward(dist)
                .build();
        startTime = .3;
        upTime = .6;
        TrajectorySequence toJunction2 = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(startTime, () -> {
                    intake.setArmMotorSpeed(-1); // arm up
                })
                .addTemporalMarker(startTime + upTime, () -> {
                    intake.setArmMotorSpeed(0);
                })
                .back(dist)
                .strafeLeft(strafe + 1.5)
                .build();
        // only if zone 2
        startTime = .8;
        downTime = .18; //?
        TrajectorySequence toStack3 = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(startTime, () -> {
                    intake.setArmMotorSpeed(1); // arm down
                })
                .addTemporalMarker(startTime + downTime, () -> {
                    intake.setArmMotorSpeed(0);
                })
                .strafeRight(strafe - 1)
                .forward(dist)
                .build();
        upTime = .21; //17; //.2; //.3;
        TrajectorySequence toJunction3 = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(startTime, () -> {
                    intake.setArmMotorSpeed(-1); // arm up
                })
                .addTemporalMarker(startTime + upTime, () -> {
                    intake.setArmMotorSpeed(0);
                })
                .back(dist)
                .strafeLeft(strafe)
                .build();
        final int armDownAtEndTime = 1000; //950;

        // april tag stuff
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "backWebcam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    for (int i = 0; i < 3; i++) {
                        if (tag.id == TAGS[i]) {
                            tagOfInterest = tag;
                            tagFound = true;
                            zone = i + 1;
                            break;
                        }
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag " + tagOfInterest.id + " found. Zone " + zone + "\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Start of Auto */
        //intake.close();

        drive.followTrajectorySequence(toLowJunction0); // move to low junction
        intake.open();  // drop 1st cone
        // 2nd cone
        drive.followTrajectorySequence(toStack1); // move to cone stack
        armDown(intake, 410); //15); //25); //400); //450);
        intake.close(); // pick up cone
        sleep(300);
        armUp(intake, 300); // lift cone
        drive.followTrajectorySequence(toJunction1); // move to low junction
        intake.open();  // drop cone
        // 3rd cone
        drive.followTrajectorySequence(toStack2); // move to cone stack
        intake.close(); // grab cone
        sleep(300);
        armUp(intake, 300); // lift cone
        drive.followTrajectorySequence(toJunction2);    // move to junction
        intake.open();  // drop cone

        // if zone is 2, get one more cone
        if(zone == 2) {
            drive.followTrajectorySequence(toStack3);
            intake.close(); // grab 4th cone
            sleep(300);
            armUp(intake, 300); // lift cone
            drive.followTrajectorySequence(toJunction3);
            intake.open(); // drop

            // park
            Trajectory traj = drive.trajectoryBuilder(startPose).back(5).build();
            drive.followTrajectory(traj);
        } else {    // park if zone is 1 or 3
            TrajectorySequence traj2 = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(strafe)
                .forward(zone == 1 ? 15.6 : -24)
                .turn(Math.toRadians(90)) // turn left
                .forward(7)
                .build();
            drive.followTrajectorySequence(traj2);
        }
        armDown(intake, armDownAtEndTime);
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID = %d (ZONE %d)", detection.id, zone));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
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
