package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingLeft {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -62, Math.toRadians(0)))
                                //first cone
                                .lineTo(new Vector2d(-36, -23.8))
                                .forward(2)
                                .waitSeconds(1)
                                .back(2)
                                //ready for cones
                                .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(180)))
                                .lineTo(new Vector2d(-56, -12))
                                .waitSeconds(1)
                                //first stack cone
                                .lineToLinearHeading(new Pose2d(-47.5, -12, Math.toRadians(270)))
                                .forward(2)
                                .waitSeconds(1)
                                .back(2)
                                .lineToLinearHeading(new Pose2d(-56, -12, Math.toRadians(180)))
                                .waitSeconds(1)
                                //second stack cone
                                .lineToLinearHeading(new Pose2d(-47.5, -12, Math.toRadians(270)))
                                .forward(2)
                                .waitSeconds(1)
                                .back(2)
                                .lineToLinearHeading(new Pose2d(-56, -12, Math.toRadians(180)))
                                .waitSeconds(1)
                                //third stack cone
                                .lineToLinearHeading(new Pose2d(-47.5, -12, Math.toRadians(270)))
                                .forward(2)
                                .waitSeconds(1)
                                .back(2)
                                .lineToLinearHeading(new Pose2d(-56, -12, Math.toRadians(180)))
                                .waitSeconds(1)
                                //fourth stack cone
                                .lineToLinearHeading(new Pose2d(-47.5, -12, Math.toRadians(270)))
                                .forward(2)
                                .waitSeconds(1)
                                .back(2)
                                .lineToLinearHeading(new Pose2d(-56, -12, Math.toRadians(180)))
                                .waitSeconds(1)
                                //fifth stack cone
                                .lineToLinearHeading(new Pose2d(-47.5, -12, Math.toRadians(270)))
                                .forward(2)
                                .waitSeconds(1)
                                .back(2)
                                .lineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(0)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}