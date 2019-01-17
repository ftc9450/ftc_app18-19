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
    private double FB_LEFT_POWER = 1.0; // TODO: check value
    private double FB_RIGHT_POWER = 1.0; // TODO: check value
    private double LR_FRONT_POWER = 1.0; // TODO: check value
    private double LR_REAR_POWER = 1.0; // TODO: check value
	private boolean rfForward;
	private boolean rbForward;
	private boolean lfForward;
	private boolean lbForward;
	private boolean rbPressed;
	private boolean lbPressed;
	private Mode mode = Mode.FB_LEFT;
	public enum Mode{
	    FB_LEFT,FB_RIGHT,LR_FRONT,LR_REAR
    }
	//private DcMotor lateral;
	//private DcMotor forward;

	public void init() {
		lb = hardwareMap.dcMotor.get(Constants.Drivetrain.LB);
		lf = hardwareMap.dcMotor.get(Constants.Drivetrain.LF);
		rb = hardwareMap.dcMotor.get(Constants.Drivetrain.RB);
		rf = hardwareMap.dcMotor.get(Constants.Drivetrain.RF);
        lb.setDirection(DcMotorSimple.Direction.FORWARD);
        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
		float z = gamepad1.right_stick_x + (gamepad1.right_trigger - gamepad1.left_trigger)/2;;
		lf.setPower(x*LR_FRONT_POWER + y*FB_LEFT_POWER + z);
		lb.setPower(-x*LR_REAR_POWER + y*FB_LEFT_POWER + z);
		rf.setPower(-x*LR_FRONT_POWER + y*FB_RIGHT_POWER - z);
		rb.setPower(x*LR_REAR_POWER + y*FB_RIGHT_POWER - z);

		/*
        if (gamepad1.x){
            lf.setPower(0.5);
        } else if(gamepad1.y) {
            lb.setPower(0.5);
        } else if(gamepad1.a){
            rf.setPower(0.5);
        } else if(gamepad1.b){
            rb.setPower(0.5);
        }
        */

        if (gamepad1.x){
            mode = Mode.FB_LEFT;
        } else if(gamepad1.y) {
            mode = Mode.FB_RIGHT;
        } else if(gamepad1.b){
            mode = Mode.LR_FRONT;
        } else if(gamepad1.a){
            mode = Mode.LR_REAR;
        }

        if(gamepad1.right_bumper){
            if(!rbPressed){
                switch(mode){
                    case FB_LEFT:
                        FB_LEFT_POWER += 0.01;
                        break;
                    case FB_RIGHT:
                        FB_RIGHT_POWER += 0.01;
                        break;
                    case LR_FRONT:
                        LR_FRONT_POWER += 0.01;
                        break;
                    case LR_REAR:
                        LR_REAR_POWER += 0.01;
                }
            }
            rbPressed = true;
        } else{
            rbPressed = false;
        }
        if(gamepad1.left_bumper){
            if(!lbPressed){
                switch(mode){
                    case FB_LEFT:
                        FB_LEFT_POWER -= 0.01;
                        break;
                    case FB_RIGHT:
                        FB_RIGHT_POWER -= 0.01;
                        break;
                    case LR_FRONT:
                        LR_FRONT_POWER -= 0.01;
                        break;
                    case LR_REAR:
                        LR_REAR_POWER -= 0.01;
                }
            }
            lbPressed = true;
        } else{
            lbPressed = false;
        }
        /*
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
        */

        /*
        telemetry.addData("right front fwd:", rfForward);
        telemetry.addData("right back fwd:", rbForward);
        telemetry.addData("left front fwd:", lfForward);
        telemetry.addData("left back fwd:", lbForward);
        */


		telemetry.addData("Mode:", mode);
		telemetry.addData("FB_LEFT_POWER:", FB_LEFT_POWER);
		telemetry.addData("FB_RIGHT_POWER:", FB_RIGHT_POWER);
		telemetry.addData("LR_FRONT_POWER:", LR_FRONT_POWER);
		telemetry.addData("LR_REAR_POWER:", LR_REAR_POWER);
		telemetry.update();

	}
}