package org.firstinspires.ftc.team9450.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.EyewearUserCalibrator;

import org.firstinspires.ftc.team9450.util.Constants;

public class Climber extends Subsystem{
    private DcMotor elevator;

    private ElevatorState elevatorState;

    public enum ElevatorState{
        UP,DOWN,OFF
    }

    public Climber(HardwareMap map){
        elevator = map.dcMotor.get(Constants.Climber.EL);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setDirection(DcMotor.Direction.FORWARD);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
                elevator.setPower(0.5);
                break;
            case DOWN:
                elevator.setPower(-0.5);
                break;
            case OFF:
                elevator.setPower(0);
                break;
        }
    }

    public void land(){
        setElevatorState(ElevatorState.UP);
        loop();
        while(getPosition() < Constants.Climber.POSITION_UP - 50){}
        setElevatorState(ElevatorState.OFF);
        stop();
    }
    public void stop() {
        elevator.setPower(0);
    }
    public void disconnectEncoders() {
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
