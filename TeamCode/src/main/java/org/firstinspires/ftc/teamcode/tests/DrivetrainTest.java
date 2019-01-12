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
	private boolean rfForward;
	private boolean rbForward;
	private boolean lfForward;
	private boolean lbForward;
	//private DcMotor lateral;
	//private DcMotor forward;

	public void init() {
		lb = hardwareMap.dcMotor.get(Constants.Drivetrain.LB);
		lf = hardwareMap.dcMotor.get(Constants.Drivetrain.LF);
		rb = hardwareMap.dcMotor.get(Constants.Drivetrain.RB);
		rf = hardwareMap.dcMotor.get(Constants.Drivetrain.RF);
        lb.setDirection(DcMotorSimple.Direction.FORWARD);
        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        rb.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rfForward = true; rbForward = true; lfForward = true; lbForward = true;
        /*
		lb = hardwareMap.dcMotor.get(Constants.MotionTracker.LR);
		rf = hardwareMap.dcMotor.get(Constants.MotionTracker.FB);
		lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		*/
	}

	public void loop() {
		double x = gamepad1.left_stick_x + (gamepad1.dpad_left? -0.5: gamepad1.dpad_right? 0.5:0);;
		double y = -gamepad1.left_stick_y + (gamepad1.dpad_down? -0.5: gamepad1.dpad_up? 0.5:0);
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
        if (gamepad1.dpad_up) {
            if (rfForward) rf.setDirection(DcMotorSimple.Direction.REVERSE);
            else rf.setDirection(DcMotorSimple.Direction.FORWARD);
            rfForward = !rfForward;
        } else if (gamepad1.dpad_right) {
            if (rbForward) rb.setDirection(DcMotorSimple.Direction.REVERSE);
            else rf.setDirection(DcMotorSimple.Direction.FORWARD);
            rbForward = !rbForward;
        } else if (gamepad1.dpad_left) {
            if (lfForward) lf.setDirection(DcMotorSimple.Direction.REVERSE);
            else  lf.setDirection(DcMotorSimple.Direction.FORWARD);
            lfForward = !lfForward;
        } else if (gamepad1.dpad_down) {
            if (lbForward) lb.setDirection(DcMotorSimple.Direction.REVERSE);
            else lb.setDirection(DcMotorSimple.Direction.FORWARD);
            lbForward = !lbForward;
        }
        telemetry.addData("right front fwd:", rfForward);
        telemetry.addData("right back fwd:", rbForward);
        telemetry.addData("left front fwd:", lfForward);
        telemetry.addData("left back fwd:", lbForward);

        /*
		telemetry.addData("lateral:", rf.getCurrentPosition());
		telemetry.addData("forward:", lb.getCurrentPosition());
		telemetry.update();
		*/
	}
}