package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Constants;

@TeleOp
public class Calibration extends OpMode{
    DcMotor elevator;
    DcMotor climberWinch;
    DcMotor pivot;
    DcMotor intakeWinch;
    Servo marker;
    public void init() {
        elevator=hardwareMap.dcMotor.get(Constants.Lifter.LIFT);
        climberWinch=hardwareMap.dcMotor.get(Constants.Climber.EL);
        pivot=hardwareMap.dcMotor.get(Constants.Intake.PI);
        intakeWinch=hardwareMap.dcMotor.get(Constants.Intake.RO);
        //marker=hardwareMap.servo.get(Constants.Auto.PIVOT);
    }

    @Override
    public void loop() {

    }
}
