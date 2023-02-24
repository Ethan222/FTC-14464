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
        Pose2d startPose = new Pose2d(-35, -61, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        RevControlHub intake = new RevControlHub();
        intake.init(hardwareMap);
        intake.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int lowPsn = 1750, highPsn = 4300, downPsn = 15;
        int[] stackArmPsns = {0, 590, 450, 310, 170, 30};
        double stackX = -53, stackY = -11.7;
        double junctionX = -18, junctionY = -9;
        double intakeWaitTime = .3; // time in seconds to wait while clamping or unclamping the intake
        double liftWaitTime = .01; // time in seconds to wait when lifting a cone off the stack
        double armDownWaitTime = .5; // time in seconds to wait before moving the arm down
        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    intake.close();
                })
                // move to low junction
                .addTemporalMarker(intakeWaitTime, () -> {
                    // arm up to low junction height
                    intake.armRuntoPosition(lowPsn);
                })
                .lineTo(new Vector2d(-31, -19))
                // drop 0th cone
                .addTemporalMarker(() -> {
                    intake.open();
                })
                .waitSeconds(intakeWaitTime)

                // 1ST CONE - move to stack 1st time
                .lineTo(new Vector2d(-35, -13))
                .addTemporalMarker(() -> {
                    // arm down to stack height (5 cones)
                    //intake.armRuntoPosition(stackArmPsns[1]);
                })
                .lineTo(new Vector2d(stackX, stackY))
                // pick up 1st cone
                .addTemporalMarker(() -> {
                    intake.close();
                })
                .waitSeconds(intakeWaitTime)
                // move to high junction 1st time
                .addTemporalMarker(() -> {
                    // arm up to high junction height
                    //intake.armRuntoPosition(highPsn);
                })
                //.waitSeconds(liftWaitTime) // wait to lift cone off stack before moving
                .lineToLinearHeading(new Pose2d(junctionX, junctionY, Math.toRadians(90)))
                // drop 1st cone
                .addTemporalMarker(() -> {
                    intake.open();
                })
                .waitSeconds(intakeWaitTime)
                /*
                // 2ND CONE - move to stack 2nd time
                .UNSTABLE_addTemporalMarkerOffset(armDownWaitTime, () -> {
                    // arm down to stack height (4 cones)
                    intake.armRuntoPosition(stackArmPsns[2]);
                })
                .lineToLinearHeading(new Pose2d(stackX, stackY, Math.toRadians(180)))
                // pick up 2nd cone
                .addTemporalMarker(() -> {
                    intake.close();
                })
                .waitSeconds(intakeWaitTime)
                // move to high junction 2nd time
                .addTemporalMarker(() -> {
                    // arm up to high junction height
                    intake.armRuntoPosition(highPsn);
                })
                .waitSeconds(liftWaitTime) // wait to lift cone off stack before moving
                .lineToLinearHeading(new Pose2d(junctionX, junctionY, Math.toRadians(90)))
                // drop 2nd cone
                .addTemporalMarker(() -> {
                    intake.open();
                })
                .waitSeconds(intakeWaitTime)

                // 3RD CONE - move to stack 3rd time
                .UNSTABLE_addTemporalMarkerOffset(armDownWaitTime, () -> {
                    // arm down to stack height (3 cones)
                    intake.armRuntoPosition(stackArmPsns[3]);
                })
                .lineToLinearHeading(new Pose2d(stackX, stackY, Math.toRadians(180)))
                // pick up 3rd cone
                .addTemporalMarker(() -> {
                    intake.close();
                })
                .waitSeconds(intakeWaitTime)
                // move to high junction 3rd time
                .addTemporalMarker(() -> {
                    // arm up to high junction height
                    intake.armRuntoPosition(highPsn);
                })
                .waitSeconds(liftWaitTime) // wait to lift cone off stack before moving
                .lineToLinearHeading(new Pose2d(junctionX, junctionY, Math.toRadians(90)))
                // drop 3rd cone
                .addTemporalMarker(() -> {
                    intake.open();
                })
                .waitSeconds(intakeWaitTime)

                // 4TH CONE - move to stack 4th time
                .UNSTABLE_addTemporalMarkerOffset(armDownWaitTime, () -> {
                    // arm down to stack height (2 cones)
                    intake.armRuntoPosition(stackArmPsns[4]);
                })
                .lineToLinearHeading(new Pose2d(stackX, stackY, Math.toRadians(180)))
                // pick up 4th cone
                .addTemporalMarker(() -> {
                    intake.close();
                })
                .waitSeconds(intakeWaitTime)
                // move to high junction 4th time
                .addTemporalMarker(() -> {
                    // arm up to high junction height
                    intake.armRuntoPosition(highPsn);
                })
                .waitSeconds((liftWaitTime)) // wait to lift cone off stack before moving
                .lineToLinearHeading(new Pose2d(junctionX, junctionY, Math.toRadians(90)))
                // drop 4th cone
                .addTemporalMarker(() -> {
                    intake.open();
                })
                .waitSeconds(intakeWaitTime)

                // 5TH (LAST) CONE - move to stack 5th time
                .UNSTABLE_addTemporalMarkerOffset(armDownWaitTime, () -> {
                    // arm down to stack height (1 cone)
                    intake.armRuntoPosition(stackArmPsns[5]);
                })
                .lineToLinearHeading(new Pose2d(stackX, stackY, Math.toRadians(180)))
                // pick up 5th cone
                .addTemporalMarker(() -> {
                    intake.close();
                })
                .waitSeconds(intakeWaitTime)
                // move to high junction 5th time
                .addTemporalMarker(() -> {
                    // arm up to high junction height
                    intake.armRuntoPosition(highPsn);
                })
                .waitSeconds((liftWaitTime)) // wait to lift cone off stack before moving
                .lineToLinearHeading(new Pose2d(junctionX, junctionY, Math.toRadians(90)))
                // drop 5th cone
                .addTemporalMarker(() -> {
                    intake.open();
                })
                .waitSeconds(intakeWaitTime)*/
                .build();
        TrajectorySequence zone1 = drive.trajectorySequenceBuilder(traj.end())
                .addTemporalMarker(armDownWaitTime, () -> {
                    intake.armRuntoPosition(downPsn);
                })
                .lineTo(new Vector2d(stackX, stackY))
                .build();
        TrajectorySequence zone2 = drive.trajectorySequenceBuilder(traj.end())
                .addTemporalMarker(armDownWaitTime, () -> {
                    intake.armRuntoPosition(downPsn);
                })
                .lineTo(new Vector2d(-35, -14))
                .build();
        TrajectorySequence zone3 = drive.trajectorySequenceBuilder(traj.end())
                .addTemporalMarker(armDownWaitTime, () -> {
                    intake.armRuntoPosition(downPsn);
                })
                .lineToLinearHeading(new Pose2d(-11.5, -14, Math.toRadians(-90)))
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