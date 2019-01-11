package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Constants;

@TeleOp
public class MotorTest extends OpMode {
    private DcMotor climb;
    private DcMotor lifter;
    private DcMotor extend;
    private DcMotor pivot;
    private CRServo roller;
    private Servo lid;

    @Override
    public void init() {
        lifter = hardwareMap.dcMotor.get(Constants.Lifter.LIFT);
        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*
        climb = hardwareMap.dcMotor.get(Constants.Climber.CL);
		climb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend = hardwareMap.dcMotor.get(Constants.Intake.EX);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot = hardwareMap.dcMotor.get(Constants.Intake.PI);
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        */
        //roller = hardwareMap.crservo.get(Constants.Intake.RO);
        //lid = hardwareMap.servo.get(Constants.Lifter.LID);
    }

    @Override
    public void loop() {
        /*
    	if (gamepad1.dpad_up) climb.setPower(0.2);
    	else if (gamepad1.dpad_down) climb.setPower(-0.2);
    	else climb.setPower(0);
    	telemetry.addData("climber:", climb.getCurrentPosition());
    	*/
    	if (gamepad1.right_bumper) lifter.setPower(0.2);
    	else if (gamepad1.left_bumper) lifter.setPower(-0.2);
    	else lifter.setPower(0);
    	telemetry.addData("elevator:", lifter.getCurrentPosition());
/*
        if (gamepad1.x) extend.setPower(0.2);
        else if (gamepad1.y) extend.setPower(-0.2);
        else lifter.setPower(0);
        telemetry.addData("extender:", lifter.getCurrentPosition());

        if (gamepad1.b) pivot.setPower(0.2);
        else if (gamepad1.a) pivot.setPower(-0.2);
        else lifter.setPower(0);
        telemetry.addData("pivot:", lifter.getCurrentPosition());
*/
    	telemetry.update();
    }
}