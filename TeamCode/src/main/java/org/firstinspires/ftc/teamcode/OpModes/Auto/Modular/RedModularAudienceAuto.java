package org.firstinspires.ftc.teamcode.OpModes.Auto.Modular;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Utils.Team;

@Autonomous(name = "Red Audience Auto", group = "Red", preselectTeleOp = "Red Audience TeleOp")
public class RedModularAudienceAuto extends ModularAuto {
    @Override
    protected Pose getStartingPose() {
        return PoseDatabase.RED_START;
    }

    @Override
    protected Team getTeam() {
        return Team.RED;
    }

    @Override
    protected void setRoute() {
        addStep(RouteStep.SHOOT_PRELOAD);
        addStep(RouteStep.COLLECT_SPIKE_1);
        addStep(RouteStep.SHOOT);
        addStep(RouteStep.COLLECT_SPIKE_2);
        addStep(RouteStep.SHOOT);
        addStep(RouteStep.COLLECT_SPIKE_3);
        addStep(RouteStep.SHOOT);
        addStep(RouteStep.PARK);
    }
}
