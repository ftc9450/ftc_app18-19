package org.firstinspires.ftc.team9450.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team9450.util.Constants;

public class Intake {}//extends Subsystem{
    /*private DcMotor slider;
    private Servo pivot;
    private Servo roller;
    private RollerState rollerState;
    private PivotState pivotState;
    private SlideState slideState;
    public enum RollerState{
        IN,OUT,OFF
    }
    public enum PivotState{
        UP,DOWN,OFF
    }
    public enum SlideState{
        IN,OUT,OFF
    }

    public Intake(Servo pivot, Servo roller, DcMotor slider){
        this.roller = roller;
        this.pivot = pivot;
        this.slider = slider;

        enableAndResetEncoders();
        slider.setDirection(DcMotorSimple.Direction.FORWARD);

        this.setPivotState(PivotState.OFF);
        this.setRollerState(RollerState.OFF);
        this.setSliderState(SlideState.OFF);


    }

    public void stop() {
        slider.setPower(0);
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
    public void setSliderState(SlideState state){
        slideState = state;
    }


    public void move(){
        enableAndResetEncoders();
    }
    public void vertical(){
        pivot.setPosition(Constants.Intake.PIVOT_UP);
    }

    @Override
    public void loop() {
        switch (rollerState){
            case IN:
                roller.setPower(Constants.Intake.ROLLER_IN);
                break;
            case OUT:
                roller.setPower(Constants.Intake.ROLLER_OUT);
                break;
            case OFF:
                roller.setPower(Constants.Intake.ROLLER_OFF);
                break;
        }
        switch (pivotState){
            case UP:
                pivot.setPosition(Constants.Intake.PIVOT_UP);
                break;
            case DOWN:
                pivot.setPosition(Constants.Intake.PIVOT_DOWN);
                break;
            case OFF:
                pivot.setPosition(Constants.Intake.PIVOT_OFF);
                break;
        }
        switch (slideState){
            case IN:
                slider.setPower(Constants.Intake.SlIDER_IN);
                break;
            case OUT:
                slider.setPower(Constants.Intake.SLIDER_OUT);
                break;
            case OFF:
                slider.setPower(Constants.Intake.SLIDER_OFF);
                break;
        }
    }
}*/
