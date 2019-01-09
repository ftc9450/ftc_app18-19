package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class DrivetrainTest extends OpMode {
	private DcMotor lb;
	private DcMotor lf;
	private DcMotor rb;
	private DcMotor rf;
	private DcMotor lateral;

	public void init() {
		lb = hardwareMap.dcmotor.get(Constants.Drivetrain.LB);
		lf = hardwareMap.dcmotor.get(Constants.Drivetrain.LF);
		rb = hardwareMap.dcmotor.get(Constants.Drivetrain.RB);
		rf = hardwareMap.dcmotor.get(Constants.Drivetrain.RF);
		lateral = hardwareMap.dcmotor.get(Constants.MotionTracker.LR);
		forward = hardwareMap.dcmotor.get(Constants.MotionTracker.FB);
	}

	public void loop() {
		float x = gamepad1.left_stick_x;
		float y = gamepad1.left_stick_y;
		float z = gamepad1.right_stick_x;
		lf.setPower(x + y + z);
		lb.setPower(-x + y + z);
		rf.setPower(-x + y - z);
		rb.setPower(x + y - z);

		telemetry.addData("lateral:", lateral.getPosition());
		telemetry.addData("forward:", lateral.getPosition());
	}
}