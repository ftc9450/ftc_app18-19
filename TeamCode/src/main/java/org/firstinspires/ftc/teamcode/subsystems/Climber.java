package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Climber extends Subsystem{
    private DcMotor climb;
    private ClimberState climberState;
    public enum ClimberState{
        UP,DOWN,OFF
    }
    public Climber(DcMotor cl){
        climb = cl;
        climb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        climb.setDirection(DcMotor.Direction.FORWARD);
        climb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setClimberState(ClimberState state){
        climberState = state;
    }
    public void enableAndResetEncoders() {
    }
    public void loop() {
        switch(climberState){
            case UP:
                climb.setPower(1);
                break;
            case DOWN:
                climb.setPower(-1);
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
