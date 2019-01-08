package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Constants;

public class Climber extends Subsystem{
    private DcMotor climb;
    double power = 0;
    private double maxPower;
    public String state = "up";
    public Climber(DcMotor cl){
        climb = cl;
    }
    public void enableAndResetEncoders() {

        climb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void loop() {
        changedir();
        enableAndResetEncoders();
        climb.setPower(Constants.Climber.HIGH_POWER);

    }
    public void changedir(){
        climb.setDirection(DcMotor.Direction.REVERSE);

    }
    public void stop() {
        climb.setPower(Constants.Climber.LOW_POWER);
    }
    public void disconnectEncoders() {
        climb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
