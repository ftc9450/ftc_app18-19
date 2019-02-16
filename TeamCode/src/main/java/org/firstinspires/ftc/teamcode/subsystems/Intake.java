package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Constants;

public class Intake extends Subsystem{
    private Servo roller;
    private DcMotor slide;
    private Servo pivot;
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

    public Intake(DcMotor slide, Servo roller, Servo pivot){
        this.roller = roller;
        this.pivot = pivot;
        this.slide = slide;

        enableAndResetEncoders();

        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slide.setDirection(DcMotor.Direction.FORWARD); // TODO: check direction

        this.setPivotState(PivotState.OFF);
        this.setRollerState(RollerState.OFF);
        this.setSlideState(SlideState.OFF);

    }

    public void stop() {
        pivot.setPosition(0);
        roller.setPosition(0);
        slide.setPower(0);
    }
    public void enableAndResetEncoders() {
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    public void setSlideState(SlideState state){
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
                roller.setPosition(Constants.Intake.ROllER_IN);
                break;
            case OUT:
                roller.setPosition(Constants.Intake.ROLLER_OUT);
                break;
            case OFF:
                roller.setPosition(Constants.Intake.ROLLER_OFF);
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
                pivot.setPower(Constants.Intake.SLIDE_IN);
                break;
            case OUT:
                pivot.setPower(Constants.Intake.SLIDE_OUT);
                break;
            case OFF:
                pivot.setPower(Constants.Intake.SLIDE_OFF);
                break;
        }
    }
}
