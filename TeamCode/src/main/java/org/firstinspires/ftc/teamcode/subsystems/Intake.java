package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends Subsystem{
    private Servo roller;
    private DcMotor move;
    private DcMotor vert;
    public String state = "inside";
    public enum RollerState{
        IN,OUT,OFF
    }
    private RollerState rollerState;
    public Intake(DcMotor vert, DcMotor move, Servo roller){
        roller = roller;
        move = move;
        vert = vert;
        vert.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vert.setDirection(DcMotorSimple.Direction.REVERSE); //TODO: check direction
        move.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void stop() {
        vert.setPower(0);
        move.setPower(0);

    }
    public void enableAndResetEncoders() {
        move.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void disconnectEncoders() {

        move.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void move(){
        enableAndResetEncoders();
        move.setPower(1);
        move.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void vertical(){
        vert.setPower(.5);
        vert.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setRollerState(RollerState state){
        rollerState = state;
    }

    @Override
    public void loop() {
        switch (rollerState){
            case IN:
                roller.setPosition(12);
                break;
            case OUT:
                roller.setPosition(-12);
                break;
            case OFF:
            default:
                break;
        }
    }
}
