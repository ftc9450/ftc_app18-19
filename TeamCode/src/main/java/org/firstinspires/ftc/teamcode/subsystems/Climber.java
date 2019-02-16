package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.EyewearUserCalibrator;

import org.firstinspires.ftc.teamcode.util.Constants;

public class Climber extends Subsystem{
    private DcMotor elevator;
    private ElevatorState elevatorState;
    private PivotState pivotState;
    private HookState hookState;
    private PawlState pawlState;
    public enum ElevatorState{
        UP,DOWN,OFF
    }

    public Climber(DcMotor elevator){
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator.setDirection(DcMotor.Direction.FORWARD);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.elevator = elevator;
        this.setElevatorState(ElevatorState.OFF);


    }
    public void setElevatorState(ElevatorState state){
        elevatorState = state;
    }

    public void enableAndResetEncoders() {
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public int getPosition(){return elevator.getCurrentPosition();}
    public void loop() {
        switch(elevatorState){
            case UP:
                //climb.setTargetPosition(Constants.Climber.UP);
                elevator.setPower(0.5);
                break;
            case DOWN:
                //climb.setTargetPosition(Constants.Climber.DOWN);
                elevator.setPower(-0.5);
                break;
            case OFF:
                elevator.setPower(0);
                break;
        }
    }
    public void stop() {
        elevator.setPower(0);
    }
    public void disconnectEncoders() {
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
