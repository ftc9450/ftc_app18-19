package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Constants;

public class Climber extends Subsystem{
    private DcMotor climb;
    private ClimberState climberState;
    public enum ClimberState{
        UP,DOWN,OFF
    }
    public Climber(DcMotor cl){
        climb = cl;
        climb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //climb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climb.setDirection(DcMotor.Direction.FORWARD);
        climb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.setClimberState(ClimberState.OFF);
    }
    public void setClimberState(ClimberState state){
        climberState = state;
    }
    public void enableAndResetEncoders() {
        climb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void loop() {
        switch(climberState){
            case UP:
                climb.setTargetPosition(Constants.Climber.UP);
                climb.setPower(0.5);
                break;
            case DOWN:
                climb.setTargetPosition(Constants.Climber.DOWN);
                climb.setPower(-0.5);
                break;
            case OFF:
                climb.setPower(0);
                break;
        }
    }
    public void stop() {
        climb.setPower(0);
    }
    public void disconnectEncoders() {
        climb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
