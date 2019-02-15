package org.firstinspires.ftc.team9450.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team9450.util.Constants;


public class Lifter extends Subsystem {

    private DcMotor extender;
    private Servo dump;
    double secondsElapsed = 0;
    private double maxPower = 0.75;
    private ExtendState extendState;
    private DumpState dumpState;
    private final double kp = 1.0;
    private final double ti = 0.0;
    private final double td = 0.0;
    public enum ExtendState{
        UP,DOWN,OFF
    }
    public enum DumpState{
        OPEN,CLOSED
    }
    public Lifter(DcMotor extend, Servo dumper){
        extender = extend;
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extender.setDirection(DcMotor.Direction.FORWARD);
        dump = dumper;
        this.setLidState(LidState.CLOSED);
        this.setLifterState(LifterState.OFF);
    }

    public void enableAndResetEncoders() {
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void loop() {
        switch(extendState){
            case UP:
                extender.setPower(0.75);
                //setPosition(Constants.Lifter.UP_POSITION);
                break;
            case DOWN:
                extender.setPower(-0.75);
                //setPosition(-Constants.Lifter.UP_POSITION);
                break;
            case OFF:
                extender.setPower(0);
        }
        switch(dumpState){
            case OPEN:
                dump.setPosition(Constants.Lifter.DUMP_OPEN);
                break;
            case CLOSED:
                dump.setPosition(Constants.Lifter.DUMP_CLOSED);
                break;
        }
    }
    public void stop() {
        extender.setPower(0);
    }
    public void disconnectEncoders() {
        extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
            currentDistance = extender.getCurrentPosition();
            double dt = (currentTime - previousTime) / 1000000; // In seconds
            secondsElapsed += dt;
            previousTime = currentTime;
        }
    }
    public void setExtendState(ExtendState state){
        extendState = state;
    }
    public void setLidState(DumpState state){
        dumpState = state;
    }
    public String toString(){
        return "" + extender.getCurrentPosition();
    }

}
