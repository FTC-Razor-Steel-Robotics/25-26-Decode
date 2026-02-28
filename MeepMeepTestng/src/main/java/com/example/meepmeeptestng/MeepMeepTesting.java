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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(62, -6, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(58,-6),Math.toRadians(10))
                .strafeToLinearHeading(new Vector2d(36,-6),Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(36,-45),Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(58, -6, Math.toRadians(10)), Math.toRadians(10))
                .strafeToLinearHeading(new Vector2d(49,-6),Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(49,-50),Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(58, -6, Math.toRadians(10.00)), Math.toRadians(10.00))

                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}