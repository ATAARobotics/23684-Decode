package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.UninterruptibleCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.gamepad.TriggerReader;
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;

import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.Subsystem.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystem.Transfer;
import org.firstinspires.ftc.teamcode.Utils.ShootAngle;

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
    // Buttons - Driver
    private GamepadButton turnBumper;

    // Buttons - Operator
    private GamepadButton xButton;
    private GamepadButton bButton;

    private Servo rgbServo;

    // Lynx modules for bulk caching
    private List<LynxModule> allHubs;

    double number = 0;
    double twonumber = 0;

    boolean driver = true;

    boolean loopcolour = true;

    @Override
    public void initialize() {
        // Initialize hardware and subsystems
        initializeHardware();
        rgbServo = hardwareMap.get(Servo.class, "rgbIndicator");


        // Setup buttons and commands
        setupGamepads();
        setupOperatorControls();
		setupDriverControls();

        // Register subsystems
        register(intake);
        register(shooter);
        register(spindexer);
        register(transfer);
    }


    @Override
    public void run() {
        follower.update();
        if (driver) {
           DriverControls();
           }

        if (Math.abs(operatorOp.getLeftY()) > 0.2){
            schedule(new ParallelCommandGroup(
                    spindexer.DirectPower(0.5),
                    transfer.IntakeDoorIn()
            ));
        }else{
            schedule(new ParallelCommandGroup(
                    spindexer.DirectPower(0),
                    transfer.IntakeDoorStop()
            ));
        }
        xButton.whenReleased(new InstantCommand(()->{
            if (Math.abs(operatorOp.getRightY()) < 0.2){
                schedule(transfer.TransferStop());
            }else{
                schedule(transfer.TransferIn());
            }
        }));

        if (loopcolour){

            schedule(new UninterruptibleCommand(
                    new SequentialCommandGroup(
                    new InstantCommand(()-> loopcolour = false),
                    new InstantCommand(()-> rgbServo.setPosition(0.622)),
                    new WaitCommand(1500),
                    new InstantCommand(()-> rgbServo.setPosition(0.388)),
                     new InstantCommand(()-> loopcolour = true)
            )));
        }

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

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Bulk Caching", "MANUAL mode enabled on " + allHubs.size() + " hub(s)");
        telemetry.update();
    }

    private void setupGamepads() {
        driverOp = new GamepadEx(gamepad1);
        operatorOp = new GamepadEx(gamepad2);
    }

	private void DriverControls() {
		follower.setTeleOpDrive(
				driverOp.getLeftY(),
				driverOp.getLeftX(),
				driverOp.getRightX(),
				false // Robot centric
		);
	}



    private void setupDriverControls() {
        // Turn Bumper: Toggle driver/operator control
        turnBumper = new GamepadButton(driverOp, GamepadKeys.Button.RIGHT_BUMPER);
        turnBumper.whenPressed(new SequentialCommandGroup(
                new InstantCommand(()-> driver = false),
                new TurnToCommand(follower, ShootAngle.calculateShotAngle(follower.getPose().getX(),follower.getPose().getY(),0,144)),
                new InstantCommand(()-> driver = true)
        ));
        turnBumper.whenReleased(new InstantCommand(()-> follower.startTeleOpDrive()));

    }

    private void setupOperatorControls() {
        // X Button: Manual transfer control
        xButton = new GamepadButton(operatorOp, GamepadKeys.Button.X);
        xButton.whenPressed(transfer.TransferIn());
        //xButton.whenReleased(transfer.TransferStop());

        // B Button: Intake door and intake control
        bButton = new GamepadButton(operatorOp, GamepadKeys.Button.B);
        bButton.whenPressed(() -> {
            schedule(transfer.IntakeDoorStop());
            schedule(intake.Out());
        });
        bButton.whenReleased(() -> {
            schedule(transfer.IntakeDoorStop());
            schedule(intake.Stop());
        });


		TriggerReader leftTriggerReader = new TriggerReader(
				operatorOp, GamepadKeys.Trigger.LEFT_TRIGGER
		);

		Trigger leftTrigger = new Trigger(leftTriggerReader::isDown);
		leftTrigger.whileActiveContinuous(intake.In());

		TriggerReader rightTriggerReader = new TriggerReader(
				operatorOp, GamepadKeys.Trigger.RIGHT_TRIGGER
		);

		Trigger rightTrigger = new Trigger(rightTriggerReader::isDown);
        if(rightTriggerReader.isDown()){
            telemetry.addLine("pressed");
        }
		rightTrigger.whileActiveContinuous(shooter.SetTarget(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM));
    }
}
