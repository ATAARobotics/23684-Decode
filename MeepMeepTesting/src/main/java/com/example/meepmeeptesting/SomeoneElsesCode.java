package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class SomeoneElsesCode {


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        Pose2d beginPose = new Pose2d(62,-12,Math.toRadians(180));
        Pose2d launchPose = new Pose2d(58,-12,Math.toRadians(201));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(beginPose)
                .splineToLinearHeading(launchPose,0)

                .splineTo(new Vector2d(36, -32), Math.toRadians(-90))
                .lineToY(-46)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(58,-18,Math.toRadians(200)), Math.toRadians(20))

                .splineTo(new Vector2d(58, -38), Math.toRadians(-90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}