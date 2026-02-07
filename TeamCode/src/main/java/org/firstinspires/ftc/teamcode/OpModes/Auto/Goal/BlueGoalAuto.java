package org.firstinspires.ftc.teamcode.OpModes.Auto.Goal;


import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utils.Team;

@Autonomous(name = "Blue Goal Auto", group = "Blue")
public class BlueGoalAuto extends GoalAuto {
    @Override
    protected Pose getStartingPose() {
        return new Pose(21.307, 124.098, Math.toRadians(324));
    }

    @Override
    protected Team getTeam() {
        return Team.BLUE;
    }
}
