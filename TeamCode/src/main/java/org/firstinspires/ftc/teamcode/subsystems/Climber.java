package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Climber extends Subsystem{
    private DcMotor elevator;
    private DcMotor pivot;
    private Servo hook;
    private ElevatorState elevatorState;
    private PivotState pivotState;
    public enum ElevatorState{
        UP,DOWN,OFF
    }
    public enum PivotState{
        UP,DOWN,OFF
    }
    public Climber(DcMotor elevator, DcMotor pivot, Servo hook){
        this.elevator = elevator;
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator.setDirection(DcMotor.Direction.FORWARD);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.setElevatorState(ElevatorState.OFF);
        this.pivot = pivot;
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setDirection(DcMotor.Direction.FORWARD);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.setPivotState(PivotState.OFF);
        this.hook = hook;
        //TODO: servo stuff

    }
    public void setElevatorState(ElevatorState state){
        elevatorState = state;
    }
    public void setPivotState(PivotState state){
        pivotState = state;
    }
    public void enableAndResetEncoders() {
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
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
        switch(pivotState){
            case UP:
                pivot.setPower(0.5);
                break;
            case DOWN:
                pivot.setPower(-0.5);
                break;
            case OFF:
                pivot.setPower(0);
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
