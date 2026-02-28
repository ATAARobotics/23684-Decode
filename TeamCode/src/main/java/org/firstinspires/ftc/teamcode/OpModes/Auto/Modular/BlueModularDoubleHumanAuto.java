package org.firstinspires.ftc.teamcode.OpModes.Auto.Modular;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Utils.Team;

@Autonomous(name = "Blue Modular Double Human Auto", group = "Blue", preselectTeleOp = "Blue TeleOp")
public class BlueModularDoubleHumanAuto extends ModularAuto {
    @Override
    protected Pose getStartingPose() {
        return PoseDatabase.BLUE_START;
    }

    @Override
    protected Team getTeam() {
        return Team.BLUE;
    }

    @Override
    protected void setRoute() {
        addStep(RouteStep.SHOOT_PRELOAD);
        addStep(RouteStep.COLLECT_SPIKE_1);
        addStep(RouteStep.SHOOT);
        addStep(RouteStep.COLLECT_HUMAN_PLAYER_WIGGLE);
        addStep(RouteStep.SHOOT);
        addStep(RouteStep.COLLECT_HUMAN_PLAYER_WIGGLE);
        addStep(RouteStep.SHOOT);
        addStep(RouteStep.PARK);
    }
}
