package org.firstinspires.ftc.team9450.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team9450.util.Constants;


public class Lifter extends Subsystem {

    private DcMotor lift;
    private Servo lid;
    double secondsElapsed = 0;
    private double maxPower = 0.75;
    private LifterState lifterState;
    private LidState lidState;
    private final double kp = 1.0;
    private final double ti = 0.0;
    private final double td = 0.0;
    public enum LifterState{
        UP,DOWN,OFF
    }
    public enum LidState{
        OPEN,CLOSED
    }
    public Lifter(DcMotor lifter, Servo bucketLid){
        lift = lifter;
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setDirection(DcMotor.Direction.FORWARD); // TODO: check direction
        lid = bucketLid;
        this.setLidState(LidState.CLOSED);
        this.setLifterState(LifterState.OFF);
    }

    public void enableAndResetEncoders() {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void loop() {
        switch(lifterState){
            case UP:
                lift.setPower(0.75);
                //setPosition(Constants.Lifter.UP_POSITION);
                break;
            case DOWN:
                lift.setPower(-0.75);
                //setPosition(-Constants.Lifter.UP_POSITION);
                break;
            case OFF:
                lift.setPower(0);
        }
            switch(lidState){
                case OPEN:
                lid.setPosition(Constants.Lifter.LID_OPEN);
                break;
            case CLOSED:
                lid.setPosition(Constants.Lifter.LID_CLOSED);
                break;
        }
    }
    public void stop() {
        lift.setPower(0);
    }
    public void disconnectEncoders() {
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setPosition(int targetDistance){
        enableAndResetEncoders();
        double power;
        int currentDistance = 0;
        secondsElapsed = 0;
        double previousTime = System.nanoTime();
        double currentTime;
        while(Math.abs(currentDistance) < Math.abs(targetDistance - Constants.Lifter.PID_THRESHOLD)){
            currentTime = System.nanoTime();
            currentDistance = lift.getCurrentPosition();
            double dt = (currentTime - previousTime) / 1000000; // In seconds
            secondsElapsed += dt;
            previousTime = currentTime;
        }
    }
    public void setLifterState(LifterState state){
        lifterState = state;
    }
    public void setLidState(LidState state){
        lidState = state;
    }
    public String toString(){
        return "" + lift.getCurrentPosition();
    }

}
