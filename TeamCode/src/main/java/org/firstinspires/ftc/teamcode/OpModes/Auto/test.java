package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.PedroPathing.Constants;


@TeleOp
public class test extends OpMode {
    Follower follower;

    PathChain pathChain;

    CommandScheduler scheduler;

    @Override
    public void init() {

        scheduler.reset();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(62.6875,8.875,Math.toRadians(270)));

        pathChain = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(62.6875,8.875), new Pose(72, 72))
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(270),
                        Math.toRadians(0)
                )
                .build();
    }

    @Override
    public void start(){
        scheduler.schedule(
                new FollowPathCommand(follower, pathChain)
        );
    }



    @Override
    public void loop() {

    }
}
