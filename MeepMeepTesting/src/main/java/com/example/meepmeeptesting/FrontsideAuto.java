package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class FrontsideAuto {

    public static double shootingX = 57, shootingY = -23; // this is the position used shooting blank right now
    public static double  Goalx = -60, Goaly = -60; // this is the position of the goal blank right now

    public static double AngleofShot(double x, double y){
        double diffx = Goalx - x;
        double diffy = Goaly - y;

        double angle = Math.atan2(-diffy, -diffx);
        return angle;
    }
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, -9,Math.toRadians(0)))

                // PRE LOAD

                // most likey we would start the camera init now
                .strafeToLinearHeading(new Vector2d(shootingX, shootingY), AngleofShot(shootingX,shootingY)) // move to shoot
                // checks the apriltag to find corect order
                .waitSeconds(2) // fires the artifact in corect order

                // SPIKE MARK ONE

                .strafeToLinearHeading(new Vector2d(35,-23), Math.toRadians(270)) // turns to the first spike zone
                // the brush would also start running
                .waitSeconds(0.03)
                .strafeTo(new Vector2d(35,-50)) // pick up the first spike zone
                .strafeToLinearHeading(new Vector2d(shootingX, shootingY), AngleofShot(shootingX,shootingY)) // goes to
                // shoot the brand new loaded
                .waitSeconds(2) // fires the artifact in corect order

                // SPIKE MARK TWO

                .strafeToLinearHeading(new Vector2d(10,-23), Math.toRadians(270))
                .waitSeconds(0.03)
                .strafeTo(new Vector2d(11,-50)) // pick up the Second spike zone
                .strafeToLinearHeading(new Vector2d(shootingX, shootingY), AngleofShot(shootingX,shootingY))// goes to
                // shoot the brand new loaded
                .waitSeconds(2) // fires the artifact in corect order


                // SPIKE MARK THREE

                .strafeToLinearHeading(new Vector2d(-10,-23), Math.toRadians(270))
                .waitSeconds(0.03)
                .strafeTo(new Vector2d(-11,-50)) // pick up the three spike zone
                .strafeToLinearHeading(new Vector2d(shootingX, shootingY), AngleofShot(shootingX,shootingY))// goes to
                // shoot the brand new loaded
                .waitSeconds(2) // fires the artifact in corect order

                // Human Player

                .strafeToLinearHeading(new Vector2d(57,-37), Math.toRadians(270))
                .waitSeconds(0.03)
                .strafeTo(new Vector2d(57,-56)) // pick up the three spike zone
                .strafeToLinearHeading(new Vector2d(shootingX, shootingY), AngleofShot(shootingX,shootingY))// goes to
                // shoot the brand new loaded




                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}