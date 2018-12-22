package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Constants;

@TeleOp

public class Lifter extends Subsystem {

    private DcMotor lift;
    double power = 0;
    private double maxPower;
    public Lifter(DcMotor li){
        lift = li;
    }
    public void setPower(double power) {
        lift.setPower(power);
        maxPower= Constants.Lifter.HIGH_POWER;
        lift.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    public void enableAndResetEncoders() {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void loop() {
        setPower(power);
    }
    public void stop() {
        lift.setPower(0);
    }
    public void disconnectEncoders() {
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
