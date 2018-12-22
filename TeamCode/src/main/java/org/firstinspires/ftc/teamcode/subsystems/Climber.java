package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Constants;

public class Climber {
    private DcMotor climb;
    double power = 0;
    private double maxPower;
    public Climber(DcMotor cl){
        climb = cl;
    }
    public void enableAndResetEncoders() {
        climb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void loop() {
        climb.setPower(Constants.Climber.HIGH_POWER);
    }
    public void stop() {
        climb.setPower(Constants.Climber.LOW_POWER);
    }
    public void disconnectEncoders() {
        climb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
