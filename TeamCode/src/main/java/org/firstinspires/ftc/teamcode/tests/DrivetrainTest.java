package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Constants;

@TeleOp
public class DrivetrainTest extends OpMode {
	private DcMotor lb;
	private DcMotor lf;
	private DcMotor rb;
	private DcMotor rf;
	//private DcMotor lateral;
	//private DcMotor forward;

	public void init() {
		lb = hardwareMap.dcMotor.get(Constants.Drivetrain.LB);
		lf = hardwareMap.dcMotor.get(Constants.Drivetrain.LF);
		rb = hardwareMap.dcMotor.get(Constants.Drivetrain.RB);
		rf = hardwareMap.dcMotor.get(Constants.Drivetrain.RF);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
		lb = hardwareMap.dcMotor.get(Constants.MotionTracker.LR);
		rf = hardwareMap.dcMotor.get(Constants.MotionTracker.FB);
		lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}

	public void loop() {
		float x = gamepad1.left_stick_x;
		float y = gamepad1.left_stick_y;
		float z = gamepad1.right_stick_x;
		lf.setPower(x + y + z);
		lb.setPower(-x + y + z);
		rf.setPower(-x + y - z);
		rb.setPower(x + y - z);
        if (gamepad1.x){
            lf.setPower(0.5);
        } else if(gamepad1.y) {
            lb.setPower(0.5);
        } else if(gamepad1.a){
            rf.setPower(0.5);
        } else if(gamepad1.b){
            rb.setPower(0.5);
        }

		telemetry.addData("lateral:", rf.getCurrentPosition());
		telemetry.addData("forward:", lb.getCurrentPosition());
		telemetry.update();
	}
}