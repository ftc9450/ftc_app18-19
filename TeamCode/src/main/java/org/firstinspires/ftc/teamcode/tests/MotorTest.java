package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Constants;

@TeleOp
public class MotorTest extends OpMode {
    private DcMotor climbMotor;
    private DcMotor lifter;
    @Override
    public void init() {
        climbMotor = hardwareMap.dcMotor.get(Constants.Climber.CL);
		climbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		lifter = hardwareMap.dcMotor.get(Constants.Lifter.LIFT);
		lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
    	if (gamepad1.dpad_up) climbMotor.setPower(0.2);
    	else if (gamepad1.dpad_down) climbMotor.setPower(-0.2);
    	else climbMotor.setPower(0);
    	telemetry.addData("climber:", climbMotor.getCurrentPosition());

    	if (gamepad1.right_bumper) lifter.setPower(0.2);
    	else if (gamepad1.left_bumper) lifter.setPower(-0.2);
    	else lifter.setPower(0);
    	telemetry.addData("elevator:", lifter.getCurrentPosition());

    	telemetry.update();
    }
}