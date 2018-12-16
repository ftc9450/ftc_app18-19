package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Intake extends Subsystem{
    private DcMotor roller;
    public enum RollerState{
        IN,OUT,OFF
    }
    private RollerState rollerState;
    public Intake(DcMotor roller){
        this.roller = roller;

        roller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        roller.setDirection(DcMotorSimple.Direction.REVERSE); //TODO: check direction
    }

    @Override
    public void stop() {

    }
    public void setRollerState(RollerState state){
        rollerState = state;
    }

    @Override
    public void loop() {
        switch (rollerState){
            case IN:
                roller.setPower(-1);
                break;
            case OUT:
                roller.setPower(1);
                break;
            case OFF:
            default:
                break;
        }
    }
}
