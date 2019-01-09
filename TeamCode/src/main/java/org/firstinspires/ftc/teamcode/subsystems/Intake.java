package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Intake extends Subsystem{
    private CRServo roller;
    private DcMotor extender;
    private DcMotor pivot;
    private RollerState rollerState;
    private ExtenderState extenderState;
    private PivotState pivotState;
    public enum RollerState{
        IN,OUT,OFF
    }
    public enum ExtenderState{
        IN,OUT,OFF
    }
    public enum PivotState{
        UP,DOWN,OFF
    }
    public Intake(DcMotor pivot, DcMotor extender, CRServo roller){
        this.roller = roller;
        this.extender = extender;
        this.pivot = pivot;

        enableAndResetEncoders();
        pivot.setDirection(DcMotorSimple.Direction.REVERSE); //TODO: check direction
        extender.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public void setExtenderPower(float power){
        extender.setPower(power);
    }

    public void stop() {
        pivot.setPower(0);
        extender.setPower(0);
        roller.setPower(0);
    }
    public void enableAndResetEncoders() {
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void disconnectEncoders() {
        extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setExtenderState(ExtenderState state){
        extenderState = state;
    }
    public void setRollerState(RollerState state){
        rollerState = state;
    }
    public void setPivotState(PivotState state){
        pivotState = state;
    }

    public void move(){
        enableAndResetEncoders();
        extender.setPower(1);
        extender.setDirection(DcMotorSimple.Direction.REVERSE);
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
        switch (extenderState){
            case IN:
                //extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //extender.setTargetPosition(Constants.Intake.IN_POSITION); // TODO: check value
                extender.setPower(-.5);
                break;
            case OUT:
                extender.setPower(.5);
                break;
            case OFF:
                extender.setPower(0);
                break;
        }
        switch (pivotState){
            case UP:
                pivot.setPower(.1);
                break;
            case DOWN:
                pivot.setPower(-.1);
                break;
            case OFF:
                pivot.setPower(0);
                break;
        }
    }
}
