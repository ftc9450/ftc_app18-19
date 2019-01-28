package org.firstinspires.ftc.team9450.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Intake extends Subsystem{
    private DcMotor roller;
    private DcMotor pivot;
    private RollerState rollerState;
    private PivotState pivotState;
    public enum RollerState{
        IN,OUT,OFF
    }
    public enum PivotState{
        UP,DOWN,OFF
    }

    public Intake(DcMotor pivot, DcMotor roller){
        this.roller = roller;
        this.pivot = pivot;

        enableAndResetEncoders();
        roller.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot.setDirection(DcMotorSimple.Direction.REVERSE); //TODO: check direction

        this.setPivotState(PivotState.OFF);
        this.setRollerState(RollerState.OFF);

    }

    public void stop() {
        pivot.setPower(0);
        roller.setPower(0);
    }
    public void enableAndResetEncoders() {
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void disconnectEncoders() {
        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setRollerState(RollerState state){
        rollerState = state;
    }
    public void setPivotState(PivotState state){
        pivotState = state;
    }

    public void move(){
        enableAndResetEncoders();
    }
    public void vertical(){
        pivot.setPower(.5);
        pivot.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        switch (rollerState){
            case IN:
                roller.setPower(1);
                break;
            case OUT:
                roller.setPower(-1);
                break;
            case OFF:
                roller.setPower(0);
                break;
        }
        switch (pivotState){
            case UP:
                pivot.setPower(.3);
                break;
            case DOWN:
                pivot.setPower(-.3);
                break;
            case OFF:
                pivot.setPower(0);
                break;
        }
    }
}
