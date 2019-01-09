package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Constants;

@TeleOp
public class MotorTest extends OpMode {
    private DcMotor climbMotor;
    private DcMotor elevator;
    @Override
    public void init() {
        climbMotor = hardwareMap.dcMotor.get(Constants.Climber.CL);
		climbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		elevator = hardwareMap.dcMotor.get(Constants.Lifter.LI);
		elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
    	if (gamepad1.dpad_up) climbMotor.setPower(0.2);
    	else if (gamepad1.dpad_down) climbMotor.setPower(-0.2);
    	else climbMotor.setPower(0);
    	telemetry.addData("climber:", climbMotor.getCurrentPosition());

    	if (gamepad1.right_bumper) elevator.setPower(0.2);
    	else if (gamepad1.left_bumper) elevator.setPower(-0.2);
    	else elevator.setPower(0);
    	telemetry.addData("elevator:", elevator.getCurrentPosition());

    	telemetry.update();
    }
}