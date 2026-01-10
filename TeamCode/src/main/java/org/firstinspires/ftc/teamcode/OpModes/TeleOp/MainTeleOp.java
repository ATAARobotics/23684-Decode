package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.PerpetualCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.gamepad.TriggerReader;

import org.firstinspires.ftc.teamcode.Drive;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.Subsystem.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystem.Transfer;

import java.util.List;

/**
 * Main TeleOp OpMode for driver control using CommandOpMode pattern.
 * <p>
 * Gamepad 1 (Driver):
 * - Left Stick: Drive forward/backward/strafe
 * - Right Stick: Turn left/right
 * <p>
 * Gamepad 2 (Operator):
 * - Left Trigger: Run intake
 * - Right Trigger: Run shooter at target RPM
 * - X Button: Manual transfer forward
 * - B Button: Intake door backward and intake out
 * - Left Joystick Y: Spindexer control
 * - Right Bumper: High power spindexer
 */
@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends CommandOpMode {

    // Subsystems
    private Intake intake;
    private Shooter shooter;
    private Spindexer spindexer;
    private Transfer transfer;
	private Follower follower;

    // Gamepads
    private GamepadEx driverOp;
    private GamepadEx operatorOp;

    // Buttons - Operator
    private GamepadButton xButton;
    private GamepadButton bButton;

    // Lynx modules for bulk caching
    private List<LynxModule> allHubs;

    @Override
    public void initialize() {
        // Initialize hardware and subsystems
        initializeHardware();

        // Setup gamepads
        setupGamepads();

        // Setup buttons and commands
        setupOperatorControls();
		setupDriverControls();

        // Register subsystems
        register(intake);
        register(shooter);
        register(spindexer);
        register(transfer);
    }

    private void initializeHardware() {
        // Get Lynx modules for bulk caching
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Initialize subsystems
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        transfer = new Transfer(hardwareMap);

		follower = Constants.createFollower(hardwareMap);
		follower.setStartingPose(new Pose(72, 72, Math.toRadians(270)));
		follower.startTeleOpDrive();
		follower.update();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Bulk Caching", "MANUAL mode enabled on " + allHubs.size() + " hub(s)");
        telemetry.update();
    }

    private void setupGamepads() {
        driverOp = new GamepadEx(gamepad1);
        operatorOp = new GamepadEx(gamepad2);
    }

	private void setupDriverControls() {



//        new PerpetualCommand(
//                new InstantCommand(()->
//		follower.setTeleOpDrive(
//				-driverOp.getLeftY(),
//				driverOp.getLeftX(),
//				driverOp.getRightX(),
//				false // Robot centric
//		)
//                )
//        );
	}

    private void setupOperatorControls() {
        // X Button: Manual transfer control
        xButton = new GamepadButton(operatorOp, GamepadKeys.Button.X);
        xButton.whenPressed(transfer.TransferIn());
        xButton.whenReleased(transfer.TransferOut());

        // B Button: Intake door and intake control
        bButton = new GamepadButton(operatorOp, GamepadKeys.Button.B);
        bButton.whenPressed(() -> {
            schedule(transfer.IntakeDoorOut());
            schedule(intake.Out());
        });
        bButton.whenReleased(() -> {
            schedule(transfer.IntakeDoorIn());
            schedule(intake.Stop());
        });

		TriggerReader leftTriggerReader = new TriggerReader(
				operatorOp, GamepadKeys.Trigger.LEFT_TRIGGER
		);

		Trigger leftTrigger = new Trigger(leftTriggerReader::isDown);
		leftTrigger.whileActiveOnce(intake.In());

		TriggerReader rightTriggerReader = new TriggerReader(
				operatorOp, GamepadKeys.Trigger.RIGHT_TRIGGER
		);

		Trigger rightTrigger = new Trigger(rightTriggerReader::isDown);
		rightTrigger.whileActiveOnce(shooter.SetTarget(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM));
    }
}
