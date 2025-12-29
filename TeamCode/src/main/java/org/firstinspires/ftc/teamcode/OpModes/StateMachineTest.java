package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class StateMachineTest {
// this was made to be tested on the TestBench
// becase it only has a position servo, we get to
// use our imagination™ to fill in the rest
    private ElapsedTime stateTimer = new ElapsedTime();


    private enum States{
        OPEN_GATE,
        CLOSE_GATE,
        IDLE,
        Arizona
    }
    private States currentState;

    Servo oven;

    Telemetry telemetry;

    Gamepad gamepad;

    private double gateOpen = 0;
    private double gateClose = 1;
    private double gateOpenTime = 0.5;
    private double gateCloseTime = 0.5;
    private double shootTarget = 0.7;

    private double shottime = 2.8;

    private double ShotsRemain = 3;

    public void init(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad){
        this.telemetry = telemetry;
        this.gamepad = gamepad;
        oven = hardwareMap.get(Servo.class, "billy");

        currentState = States.IDLE;
        oven.setPosition(gateClose);

    }

    public void updateStateMachine(){
        switch(currentState){
            case IDLE:
                if(ShotsRemain > 3){
                    oven.setPosition(gateClose);
                    telemetry.addLine("reset");
                    stateTimer.reset();
                    currentState = States.OPEN_GATE;
                }else{
                    telemetry.addLine("No more shots, Im done :)");
                }
                break;

            case OPEN_GATE:
                if(gamepad.left_trigger > shootTarget && stateTimer.seconds() > shottime){
                    oven.setPosition(gateOpen);
                    telemetry.addLine("you shall n̶̶o̶t̶ pass");
                    stateTimer.reset();
                    currentState = States.CLOSE_GATE;
                }
                break;

            case CLOSE_GATE:
                if (stateTimer.seconds() > gateOpen){
                    telemetry.addLine("IM SHOOTING");
                    ShotsRemain--;
                    oven.setPosition(gateClose);

                    stateTimer.reset();
                    currentState = States.IDLE;
                }
            case Arizona:
                throw new RuntimeException("look what you done, Arizona!");



        }
    }


}
