package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        double stackX = -58, stackY = -11.7;
        double junctionX = -22, junctionY = -12;
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(73.17330064499293, 73.17330064499293, Math.toRadians(360), Math.toRadians(360), 10.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -61, Math.toRadians(180)))
                                .lineTo(new Vector2d(-35, -24))
                                .waitSeconds(.5) // drop 0th cone

                                // move to stack 1st time
                                .lineTo(new Vector2d(-35, -13))
                                .lineTo(new Vector2d(stackX, stackY))
                                .waitSeconds(1) // pick up 1st cone
                                // move to high junction 1st time
                                .lineToLinearHeading(new Pose2d(junctionX, junctionY, Math.toRadians(90)))
                                .waitSeconds(.5) // drop 1st cone

                                // move to stack 2nd time
                                .lineToLinearHeading(new Pose2d(stackX, stackY, Math.toRadians(180)))
                                .waitSeconds(.5) // pick up 2nd cone
                                // move to high junction 2nd time
                                .lineToLinearHeading(new Pose2d(junctionX, junctionY, Math.toRadians(90)))
                                .waitSeconds(.5) // drop 2nd cone

                                // move to stack 3rd time
                                .lineToLinearHeading(new Pose2d(stackX, stackY, Math.toRadians(180)))
                                .waitSeconds(.5) // pick up 3rd cone
                                // move to high junction 3rd time
                                .lineToLinearHeading(new Pose2d(junctionX, junctionY, Math.toRadians(90)))
                                .waitSeconds(.5) // drop 3rd cone

                                // move to stack 4th time
                                .lineToLinearHeading(new Pose2d(stackX, stackY, Math.toRadians(180)))
                                .waitSeconds(.5) // pick up 4th cone
                                // move to high junction 4th time
                                .lineToLinearHeading(new Pose2d(junctionX, junctionY, Math.toRadians(90)))
                                .waitSeconds(.5) // drop 4th cone

                                // move to stack 5th time
                                .lineToLinearHeading(new Pose2d(stackX, stackY, Math.toRadians(180)))
                                .waitSeconds(.5) // pick up 5th cone
                                // move to high junction 5th time
                                .lineToLinearHeading(new Pose2d(junctionX, junctionY, Math.toRadians(90)))
                                .waitSeconds(.5) // drop 5th cone

                                // park
                                // zone 1
                                .lineToLinearHeading(new Pose2d(stackX, stackY, Math.toRadians(-90)))
                                // zone 2
                                //.lineToLinearHeading(new Pose2d(-35, -14, Math.toRadians(-90)))
                                // zone 3
                                //.lineToLinearHeading(new Pose2d(-11.5, -14, Math.toRadians(-90)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}