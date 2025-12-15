package org.firstinspires.ftc.teamcode.Utilities;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.ArrayList;
import java.util.List;

/**
 * ActionScheduler is a singleton utility that manages the scheduling and execution of Actions.
 * It allows actions to be queued and executed in the teleop loop without blocking.
 */
public class ActionScheduler {
	private static ActionScheduler instance;
	private final FtcDashboard dashboard;
	private List<Action> runningActions;

	private ActionScheduler() {
		this.dashboard = FtcDashboard.getInstance();
		this.runningActions = new ArrayList<>();
	}

	/**
	 * Get the singleton instance of ActionScheduler.
	 *
	 * @return the ActionScheduler instance
	 */
	public static synchronized ActionScheduler getInstance() {
		if (instance == null) {
			instance = new ActionScheduler();
		}
		return instance;
	}

	/**
	 * Schedule an action to be executed.
	 *
	 * @param action the action to schedule
	 */
	public void schedule(Action action) {
		runningActions.add(action);
	}

	/**
	 * Update all running actions with telemetry control.
	 * This should be called once per loop iteration.
	 * Calls run() on each active action and removes actions that are complete.
	 * 
	 * @param sendTelemetry if true, sends telemetry to FTC Dashboard; if false, skips dashboard overhead
	 */
	public void update(boolean sendTelemetry) {
		// Early exit if no actions to process
		if (runningActions.isEmpty()) {
			return;
		}

		// Only create telemetry packet if we have actions AND telemetry is enabled
		TelemetryPacket packet = null;
		List<Action> newActions = new ArrayList<>();

		for (Action action : runningActions) {
			if (sendTelemetry) {
				if (packet == null) {
					packet = new TelemetryPacket();
				}
				action.preview(packet.fieldOverlay());
				if (action.run(packet)) {
					newActions.add(action);
				}
			} else {
				// Still run actions but with null packet to skip telemetry overhead
				if (action.run(null)) {
					newActions.add(action);
				}
			}
		}

		runningActions = newActions;

		// Only send telemetry if we created a packet
		if (packet != null) {
			dashboard.sendTelemetryPacket(packet);
		}
	}

	/**
	 * Update all running actions with telemetry enabled (default behavior for backward compatibility).
	 * This should be called once per loop iteration.
	 */
	public void update() {
		update(true);
	}

	/**
	 * Update all running actions with a provided telemetry packet.
	 * This allows actions to be added to an existing packet.
	 *
	 * @param packet the telemetry packet to use
	 */
	public void update(TelemetryPacket packet) {
		List<Action> newActions = new ArrayList<>();

		for (Action action : runningActions) {
			action.preview(packet.fieldOverlay());
			if (action.run(packet)) {
				newActions.add(action);
			}
		}

		runningActions = newActions;
	}

	/**
	 * Send the current telemetry packet to the dashboard.
	 *
	 * @param packet the packet to send
	 */
	public void sendTelemetry(TelemetryPacket packet) {
		dashboard.sendTelemetryPacket(packet);
	}

	/**
	 * Get the number of currently running actions.
	 *
	 * @return the count of running actions
	 */
	public int getRunningActionCount() {
		return runningActions.size();
	}

	/**
	 * Clear all running actions.
	 */
	public void clearActions() {
		runningActions.clear();
	}

	/**
	 * Check if the action scheduler is empty.
	 *
	 * @return false if there are running actions, true otherwise
	 */
	public boolean isSchedulerEmpty() {
		return runningActions.isEmpty();
	}
}
