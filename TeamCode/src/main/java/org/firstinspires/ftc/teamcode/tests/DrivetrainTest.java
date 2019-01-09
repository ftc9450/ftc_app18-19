package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Constants;

@TeleOp
public class DrivetrainTest extends OpMode {
	private DcMotor lb;
	private DcMotor lf;
	private DcMotor rb;
	private DcMotor rf;
	private DcMotor lateral;
	private DcMotor forward;

	public void init() {
		lb = hardwareMap.dcMotor.get(Constants.Drivetrain.LB);
		lf = hardwareMap.dcMotor.get(Constants.Drivetrain.LF);
		rb = hardwareMap.dcMotor.get(Constants.Drivetrain.RB);
		rf = hardwareMap.dcMotor.get(Constants.Drivetrain.RF);
		lateral = hardwareMap.dcMotor.get(Constants.MotionTracker.LR);
		forward = hardwareMap.dcMotor.get(Constants.MotionTracker.FB);
		lateral.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		forward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	}

	public void loop() {
		float x = gamepad1.left_stick_x;
		float y = gamepad1.left_stick_y;
		float z = gamepad1.right_stick_x;
		lf.setPower(x + y + z);
		lb.setPower(-x + y + z);
		rf.setPower(-x + y - z);
		rb.setPower(x + y - z);

		telemetry.addData("lateral:", lateral.getCurrentPosition());
		telemetry.addData("forward:", forward.getCurrentPosition());
	}
}