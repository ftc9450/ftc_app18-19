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
    private DcMotor pivot;
    private DcMotor intake;
    private DcMotor deposit;

    @Override
    public void init() {
        climb = hardwareMap.dcMotor.get("climber");
		climb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		climb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		pivot = hardwareMap.dcMotor.get("pivot");
		pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        deposit = hardwareMap.dcMotor.get("deposit");
        deposit.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deposit.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = hardwareMap.dcMotor.get("roller");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {

    	if (gamepad1.dpad_up) climb.setPower(0.2);
    	else if (gamepad1.dpad_down) climb.setPower(-0.2);
    	else climb.setPower(0);
    	telemetry.addData("climber:", climb.getCurrentPosition());

    	if (gamepad1.right_trigger > 0.5) pivot.setPower(0.2);
    	else if (gamepad1.left_trigger > 0.5) pivot.setPower(-0.2);
    	else pivot.setPower(0);
    	telemetry.addData("intake pivot", pivot.getCurrentPosition());

    	if (gamepad1.right_bumper) deposit.setPower(0.2);
    	else if (gamepad1.left_bumper) deposit.setPower(-0.2);
    	else deposit.setPower(0);
    	telemetry.addData("deposit", deposit.getCurrentPosition());

    	if (gamepad1.a) {
    	    intake.setPower(1);
        } else if (gamepad1.b) {
    	    intake.setPower(-1);
        } else {
    	    intake.setPower(0);
        }

    	telemetry.update();
    }
}