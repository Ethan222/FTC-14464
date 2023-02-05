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
public class Right2 extends LinearOpMode
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

    private static final double ARM_PARTWAY_UP_TIME = .7;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        RevControlHub intake = new RevControlHub();
        intake.init(hardwareMap);

        double startTime = .5;
        TrajectorySequence toLowJunction1 = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(startTime, () -> {
                    intake.setArmMotorSpeed(-1);    // arm up
                })
                .addTemporalMarker(startTime + ARM_PARTWAY_UP_TIME, () -> {
                    intake.setArmMotorSpeed(0);     // stop arm
                })
                .forward(2)
                .strafeRight(10)
                .forward(26)
                .build();
        startTime = 1;
        //double downTime = .6;
        TrajectorySequence toStack1 = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(12)
                /*.addTemporalMarker(startTime, () -> {
                    intake.setArmMotorSpeed(1); // arm down
                })
                .addTemporalMarker(startTime + downTime, () -> {
                    intake.setArmMotorSpeed(0);
                })*/
                .forward(16)
                .turn(Math.toRadians(-90)) // turn right
                .forward(3.5)
                .build();
        double upTime = .1;
        double dist = 18.5, strafe = 14;
        TrajectorySequence toJunction2 = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(startTime, () -> {
                    intake.setArmMotorSpeed(-1); // arm up
                })
                .addTemporalMarker(startTime + upTime, () -> {
                    intake.setArmMotorSpeed(0);
                })
                .back(dist)
                .strafeRight(strafe)
                .build();
        double downTime = .6;
        TrajectorySequence toStack2 = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(startTime, () -> {
                    intake.setArmMotorSpeed(1); // arm down
                })
                .addTemporalMarker(startTime + downTime, () -> {
                    intake.setArmMotorSpeed(0);
                })
                .strafeLeft(strafe)
                .forward(dist)
                .build();
        downTime += .2;
        TrajectorySequence toStack3 = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(startTime, () -> {
                    intake.setArmMotorSpeed(1); // arm down
                })
                .addTemporalMarker(startTime + downTime, () -> {
                    intake.setArmMotorSpeed(0);
                })
                .strafeLeft(strafe + 1)
                .forward(dist)
                .build();

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
        intake.close();

        drive.followTrajectorySequence(toLowJunction1); // move to low junction
        intake.open();  // drop cone
        drive.followTrajectorySequence(toStack1); // move to cone stack
        armDown(intake, 450);
        // repeat:
        intake.close(); // pick up cone
        sleep(300);
        armUp(intake, 300); // lift cone
        drive.followTrajectorySequence(toJunction2); // move to low junction
        intake.open();  // drop cone

        drive.followTrajectorySequence(toStack2); // move to cone stack
        intake.close(); // grab cone
        sleep(300);
        armUp(intake, 300); // lift cone
        drive.followTrajectorySequence(toJunction2);
        intake.open();
        /*
        drive.followTrajectorySequence(toStack3);
        intake.close(); // grab cone
        sleep(300);
        armUp(intake, 300); // lift cone
        drive.followTrajectorySequence(toJunction2);
        intake.open();
        */
        // park
        if(zone == 2) {
            Trajectory traj = drive.trajectoryBuilder(startPose).back(5).build();
            drive.followTrajectory(traj);
        } else {
            // strafe left
            Trajectory traj1 = drive.trajectoryBuilder(startPose).strafeLeft(strafe).build();
            drive.followTrajectory(traj1);

            // move forward/backward
            Trajectory traj2;
            if(zone == 1)
                traj2 = drive.trajectoryBuilder(startPose).back(24).build();
            else
                traj2 = drive.trajectoryBuilder(startPose).forward(18).build();
            drive.followTrajectory(traj2);
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID = %d", detection.id));
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
