package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.MotionTracker;

/**
 * Created by dhruv on 1/20/18.
 */

public class Drivetrain extends Subsystem {
    private DrivetrainState state;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private double maxPower;

    public enum DrivetrainState{
        Turning, Linear
    }

    public Drivetrain(DcMotor lf, DcMotor lb, DcMotor rf, DcMotor rb) {
        leftFront = lf;
        leftBack = lb;
        rightFront = rf;
        rightBack = rb;
        maxPower=Constants.Drivetrain.HIGH_POWER;
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        maxPower = 1;
        this.setState(DrivetrainState.Linear);
    }


    public void setPower(double power) {
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);
    }
    public void setPower(double driveSignal[]) {
        double scale = maxPower;
        double diff=Math.abs(rightFront.getPower() - (driveSignal[2] * maxPower));
        if(diff > 1){
            scale *= 1 - (diff / 4.0);
        }
        leftFront.setPower(driveSignal[0] * scale);
        leftBack.setPower(driveSignal[1] * scale);
        rightFront.setPower(driveSignal[2] * scale);
        rightBack.setPower(driveSignal[3] * scale);
    }
    public void moveFB(double distance, double power, boolean forward, MotionTracker tracker){ // distance (in inches) and power will always be positive
        tracker.enableAndResetEncoders();
        double leftPower = power * Constants.Drivetrain.FB_LEFT_POWER;
        double rightPower = power * Constants.Drivetrain.FB_RIGHT_POWER;
        int clicks = (int) (distance * Constants.MotionTracker.CLICKS_PER_INCH);
        if(forward){
            setPower(new double[]{leftPower,leftPower,rightPower,rightPower});
            while( tracker.getYEncoderValue()<clicks){}
        }else{
            setPower(new double[]{-leftPower,-leftPower,-rightPower,-rightPower});
            while( tracker.getYEncoderValue()>(-clicks)){}
        }
        setPower(new double[]{0,0,0,0});
        /*
        state = DrivetrainState.Linear;
        int clicks = (int) (distance * Constants.MotionTracker.CLICKS_PER_INCH);
        int initialEncoderValue = tracker.getYEncoderValue();
        double leftPower = power * Constants.Drivetrain.FB_LEFT_POWER;
        double rightPower = power * Constants.Drivetrain.FB_RIGHT_POWER;
        if(forward){
            setPower(new double[]{leftPower,leftPower,rightPower,rightPower});
            try{while(tracker.getYEncoderValue() - initialEncoderValue < clicks - Constants.Drivetrain.FB_THRESHOLD) tracker.updatePosition();}catch(Exception e){}
        } else{
            setPower(new double[]{-leftPower,-leftPower,-rightPower,-rightPower});
            try{while(initialEncoderValue - tracker.getYEncoderValue() < clicks - Constants.Drivetrain.FB_THRESHOLD) tracker.updatePosition();}catch(Exception e){}
        }
        setPower(new double[]{0,0,0,0});
        tracker.updatePosition();*/
    }
    public void moveLR(int distance, double power, boolean right, MotionTracker tracker){//positive power and distance is move to right
        state = DrivetrainState.Linear;
        int clicks = (int)(distance * Constants.MotionTracker.CLICKS_PER_INCH);
        int initialEncoderValue = tracker.getYEncoderValue();
        double frontPower = power * Constants.Drivetrain.LR_FRONT_POWER;
        double rearPower = power * Constants.Drivetrain.LR_REAR_POWER;
        if(right){
            setPower(new double[]{frontPower,rearPower,frontPower,rearPower});
            try{while(tracker.getYEncoderValue() - initialEncoderValue < clicks - Constants.Drivetrain.LR_THRESHOLD) tracker.updatePosition();}catch(Exception e){}
        } else{
            setPower(new double[]{-frontPower,-rearPower,-frontPower,-rearPower});
            try{while(initialEncoderValue - tracker.getYEncoderValue() < clicks - Constants.Drivetrain.LR_THRESHOLD) tracker.updatePosition();}catch(Exception e){}
        }
        setPower(new double[]{0,0,0,0});
        tracker.updatePosition();
    }
    /* Now in auto classes
    public void pivotClockwise(double angle, Gyroscope imu, MotionTracker tracker){ // Turn clockwise given degree angle
        state = DrivetrainState.Turning;
        float startAngle = imu.getAngle();
        double[] driveSignal = new double[]{0.3,0.3,-0.3,-0.3};
        setPower(driveSignal);
        while(angle - (imu.getAngle() - startAngle + 360) % 360 > 0.1); // TODO: check value
        setPower(new double[]{0,0,0,0});
        tracker.updatePosition();
    }
    public void pivotCounterclockwise(double angle, Gyroscope imu, MotionTracker tracker){ // Turn counterclockwise given degree angle
        state = DrivetrainState.Turning;
        float startAngle = imu.getAngle();
        double[] driveSignal = new double[]{-0.3,-0.3,0.3,0.3};
        setPower(driveSignal);
        while(angle - (startAngle - imu.getAngle() + 360) % 360 > 0.1); // TODO: check value
        setPower(new double[]{0,0,0,0});
        tracker.updatePosition();
    }
    */
    public void setFWPosition(double pos) {
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public boolean isClose(DcMotor dcMotor){return Math.abs(dcMotor.getCurrentPosition()-dcMotor.getTargetPosition())<10;}
    public boolean isBusy(){return !(isClose(leftFront) || isClose(leftBack) || isClose(rightFront) || isClose(rightBack));}
    public void enableAndResetEncoders(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void disconnectEncoders(){
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setState(DrivetrainState state){
        this.state = state;
    }
    public DrivetrainState getState(){
        return state;
    }
    public String toString(){
        return String.valueOf((leftFront.getCurrentPosition()+rightFront.getCurrentPosition())/2 );
    }
    @Override
    public void stop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
    public int getPosition() {
        return (leftFront.getCurrentPosition() + rightFront.getCurrentPosition())/2;
    }

    @Override
    public void loop() { }
}
