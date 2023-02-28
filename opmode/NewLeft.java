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
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.RevControlHub;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous(group = "drive")
public class NewLeft extends LinearOpMode
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
        Pose2d startPose = new Pose2d(-31, -61, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        RevControlHub intake = new RevControlHub();
        intake.init(hardwareMap);
        intake.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int lowPsn = 1850, highPsn = 4470, downPsn = 15;
        int[] stackArmPsns = {0, 590, 450, 310, 170, 30};
        double stackX = -61.7, stackY = -7;
        double junctionX = -26.1-.3+.4-.3, junctionY = -6-.4+.2;
        double intakeWaitTime = .5; // time in seconds to wait while clamping or unclamping the intake
        double armDownWaitTime = .5; // time in seconds to wait before moving the arm down
        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    intake.close();
                })
                // move to high junction 1st time
                .lineTo(new Vector2d(-15, -57))
                .lineTo(new Vector2d(-15, -12))
                .addTemporalMarker(() -> {
                    intake.armRuntoPosition(highPsn, this);
                })
                .lineTo(new Vector2d(junctionX, junctionY + 2-.3+.2))
                // drop 1st cone
                .addTemporalMarker(() -> {
                    intake.open();
                })
                .waitSeconds(intakeWaitTime)

                // 2nd cone - move to stack 1st time
                //.lineToLinearHeading(new Pose2d(-35, -11, Math.toRadians(135)))
                //.lineToLinearHeading(new Pose2d(stackX, stackY, Math.toRadians(170)))
                .back(2.8)
                .turn(Math.toRadians(90))
                .addTemporalMarker(() -> {
                    // arm down to stack height (5 cones)
                    intake.armRuntoPosition(stackArmPsns[1], this);
                })
                .lineTo(new Vector2d(stackX, stackY))
                // pick up 2nd cone
                .addTemporalMarker(() -> {
                    intake.close();
                })
                .waitSeconds(intakeWaitTime)
                // move to high junction 2nd time
                .addTemporalMarker(() -> {
                    // arm up to high junction height
                    intake.armRuntoPosition(highPsn, this);
                })
                //.waitSeconds(liftWaitTime) // wait to lift cone off stack before moving
                .lineToLinearHeading(new Pose2d(junctionX - 1-.3, junctionY-.9+.3, Math.toRadians(90)))
                //.forward(2)
                // drop 2nd cone
                .addTemporalMarker(() -> {
                    intake.open();
                })
                .waitSeconds(intakeWaitTime)
                .build();
        TrajectorySequence zone1 = drive.trajectorySequenceBuilder(traj.end())
                .back(2)
                .turn(Math.toRadians(90))
                .addTemporalMarker(armDownWaitTime, () -> {
                    intake.armRuntoPosition(stackArmPsns[2], this);
                })
                .lineTo(new Vector2d(stackX-.3-.5, stackY))
                //.lineToLinearHeading(new Pose2d(stackX, stackY, Math.toRadians(180)))
                /*.addTemporalMarker(() -> {
                    intake.close();
                    intake.armRuntoPosition(900);
                })*/
                //.back(3)
                // don't turn so you don't hit the stack
                .build();
        TrajectorySequence zone2 = drive.trajectorySequenceBuilder(traj.end())
                .addTemporalMarker(armDownWaitTime, () -> {
                    intake.armRuntoPosition(downPsn, this);
                })
                .lineTo(new Vector2d(-31+.3, -7-.3))
                .turn(Math.toRadians(180))
                .build();
        TrajectorySequence zone3 = drive.trajectorySequenceBuilder(traj.end())
                .addTemporalMarker(armDownWaitTime, () -> {
                    intake.armRuntoPosition(downPsn, this);
                })
                .lineTo(new Vector2d(-23-2+1, -8))
                .turn(Math.toRadians(180))
                .build();
        TrajectorySequence[] zones = { null, zone1, zone2, zone3 };

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
                    telemetry.addLine("Tag " + tagOfInterest.id + " found. ZONE " + zone + "\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at ZONE " + zone);
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at ZONE " + zone);
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
        telemetry.addLine("ZONE " + zone);
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Start of Auto */
        ElapsedTime t = new ElapsedTime();

        drive.followTrajectorySequence(traj);

        // park
        drive.followTrajectorySequence(zones[zone]);

        while(!isStopRequested() && opModeIsActive()) {
            telemetry.addData("Time", t.time());
            telemetry.update();
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
}