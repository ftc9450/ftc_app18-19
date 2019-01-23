package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Constants;

public class Climber extends Subsystem{
    private DcMotor elevator;
    private DcMotor pivot;
    private Servo hook;
    private Servo pawl;
    private ElevatorState elevatorState;
    private PivotState pivotState;
    private HookState hookState;
    private PawlState pawlState;
    public enum ElevatorState{
        UP,DOWN,OFF
    }
    public enum PivotState{
        UP,DOWN,OFF
    }
    public enum HookState{
        OPEN, SHUT, OFF
    }
    public enum PawlState{
        ENGAGED, DISENGAGED, OFF
    }
    public Climber(DcMotor elevator, DcMotor pivot, Servo hook, Servo pawl){
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator.setDirection(DcMotor.Direction.FORWARD);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.elevator = elevator;
        this.setElevatorState(ElevatorState.OFF);

        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setDirection(DcMotor.Direction.FORWARD);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.pivot = pivot;
        this.setPivotState(PivotState.OFF);

        hook.setDirection(Servo.Direction.FORWARD);
        this.hook = hook;
        this.setHookState(HookState.OFF);

        pawl.setDirection(Servo.Direction.FORWARD);
        this.pawl=pawl;
        this.setPawlState(PawlState.OFF);

    }
    public void setElevatorState(ElevatorState state){
        elevatorState = state;
    }
    public void setPivotState(PivotState state){
        pivotState = state;
    }
    public void setHookState(HookState state){hookState=state;}
    public void setPawlState(PawlState state){pawlState=state;}
    public void enableAndResetEncoders() {
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public int getPosition(){return elevator.getCurrentPosition();}
    public void loop() {
        switch(elevatorState){
            case UP:
                //climb.setTargetPosition(Constants.Climber.UP);
                if(elevator.getCurrentPosition()<Constants.Climber.UP-500){
                    elevator.setPower(1);
                }else{
                    elevator.setPower(0);
                }
                break;
            case DOWN:
                //climb.setTargetPosition(Constants.Climber.DOWN);
                elevator.setPower(-1);
                break;
            case OFF:
                elevator.setPower(0);
                break;
        }
        switch(pivotState){
            case UP:
                pivot.setPower(0.2);
                break;
            case DOWN:
                pivot.setPower(-0.2);
                break;
            case OFF:
                pivot.setPower(0);
                break;
        }
        switch(pawlState){
            case ENGAGED:
                pawl.setPosition(Constants.Climber.PAWL_IN);
                break;
            case DISENGAGED:
                pawl.setPosition(Constants.Climber.PAWL_OUT);
                break;
            case OFF:
                break;
        }
        switch(hookState){
            case OPEN:
                hook.setPosition(Constants.Climber.HOOK_OPEN);
                break;
            case SHUT:
                hook.setPosition(Constants.Climber.HOOK_CLOSED);
                break;
            case OFF:
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
