package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class SplineAudienceAuto {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, -9,Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(55, -9), Math.toRadians(23))
                .waitSeconds(2.8)


                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(27,-24,Math.toRadians(270)),Math.toRadians(270))
                .lineToY(-46)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(55,-9,Math.toRadians(23)),Math.toRadians(23))
                .waitSeconds(2.8)

                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(4,-24,Math.toRadians(270)),Math.toRadians(270))
                .lineToY(-46)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(55,-9,Math.toRadians(23)),Math.toRadians(23))
                .waitSeconds(2.8)

                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-20,-24,Math.toRadians(270)),Math.toRadians(270))
                .lineToY(-37)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(55,-9,Math.toRadians(23)),Math.toRadians(23))



                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}