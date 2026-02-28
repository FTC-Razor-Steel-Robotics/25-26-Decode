package com.example.meepmeeptestng;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-67, 15, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(-12,15),Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-12,50),Math.toRadians(90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-23.50, 24.35, Math.toRadians(315)), Math.toRadians(315))
                .strafeToLinearHeading(new Vector2d(12,24.35),Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(12,50))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-23.50, 24.35, Math.toRadians(315)), Math.toRadians(315))

                .strafeToLinearHeading(new Vector2d(-15,24.35),Math.toRadians(350))

                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}