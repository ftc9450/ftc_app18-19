package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Constants;

public class Climber {
    private DcMotor climb;
    double power = 0;
    private double maxPower;
    public Climber(DcMotor li){
        climb = li;
    }
    public void setPower(double power) {
        climb.setPower(power);
        maxPower= Constants.Climber.HIGH_POWER;
        climb.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void enableAndResetEncoders() {
        climb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void loop() {
        climb.setPower(1);
    }
    public void stop() {
        climb.setPower(0);
    }
    public void disconnectEncoders() {
        climb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
