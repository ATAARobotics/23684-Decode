package org.firstinspires.ftc.teamcode.Utils;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Subsystem.Conveyor;
import org.firstinspires.ftc.teamcode.Subsystem.Gate;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.Subsystem.Transfer;

import java.util.Objects;

/**
 * Auto shoot sequence: prespin the shooter, wait until the shooter is at
 * target AND the robot footprint overlaps a shooting zone, open the gate,
 * transfer + conveyor out, then close.
 * <p>
 * The gate-open is gated on the shooting-zone check (same triangles used by
 * {@link RGBIndicator}) so we never feed the shooter while still in transit.
 * Both wait conditions carry a 2.5s timeout so a stale follower pose or
 * shooter that fails to reach target cannot hang the entire auto.
 */
public class ShootArtifacts extends SequentialCommandGroup {
	private static final long GATE_OPEN_TIMEOUT_MS = 2500L;

	public ShootArtifacts(Shooter shooter, Conveyor conveyor, Transfer transfer, Intake intake, Gate gate, Follower follower, Team team, int waitTime) {
		Objects.requireNonNull(follower, "follower");
		Objects.requireNonNull(team, "team");
		if (team == Team.UNKNOWN) {
			throw new IllegalArgumentException("team must be RED or BLUE");
		}

		addCommands(
				shooter.SetTarget(Shooter.AUDIENCE_RPM_UPPER, Shooter.AUDIENCE_RPM_LOWER),
				new ParallelCommandGroup(
						intake.Stop(),
						new SequentialCommandGroup(
							new WaitUntilCommand(() -> shooter.getPercentToTarget() >= 0.8
									&& isInShootingZone(follower, team))
									.withTimeout(GATE_OPEN_TIMEOUT_MS),
							gate.openGate()
						),
				shooter.WaitForTarget().withTimeout(GATE_OPEN_TIMEOUT_MS)
				).withTimeout(GATE_OPEN_TIMEOUT_MS + 500L),
				new WaitCommand(waitTime),
				transfer.TransferOut(),
				conveyor.In()
		);

		addRequirements(shooter, conveyor, transfer, intake, gate);
	}

	private static boolean isInShootingZone(Follower follower, Team team) {
		return team == Team.RED
				? ShootingZone.isAnyCornerInRedZone(follower.getPose())
				: ShootingZone.isAnyCornerInBlueZone(follower.getPose());
	}
}
