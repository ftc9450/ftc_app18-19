package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Pid;


public class Lifter extends Subsystem {

    private DcMotor extender;
    private Servo dump;

    double secondsElapsed = 0;
    private double maxPower = 0.75;
    private DumpState dumpState;
    private Pid pid = null;
    private final double kp = 1.0;
    private final double ti = 0.0;
    private final double td = 0.0;
    public enum DumpState{
        UP,DOWN
    }
    public Lifter(DcMotor lifter, Servo dumper){
        extender = lifter;
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        extender.setDirection(DcMotor.Direction.FORWARD); // TODO: check direction
        dump = dumper;
        this.setDumpState(DumpState.DOWN);
    }

    public void enableAndResetEncoders() {
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void loop() {
        switch(dumpState){
            case UP:
                dump.setPosition(Constants.Lifter.DUMP_UP);
                //setPosition(Constants.Lifter.UP_POSITION);
                break;
            case DOWN:
                dump.setPosition(Constants.Lifter.DUMP_DOWN);                //setPosition(-Constants.Lifter.UP_POSITION);
                break;

        }


    }
    public void stop() {
        lift.setPower(0);
    }
    public void disconnectEncoders() {
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /*private void setPosition(int targetDistance){
        enableAndResetEncoders();
        pid = new Pid(kp,ti,td,-1,1,-maxPower,maxPower);
        double power;
        int currentDistance = 0;
        secondsElapsed = 0;
        double previousTime = System.nanoTime();
        double currentTime;
        while(Math.abs(currentDistance) < Math.abs(targetDistance - Constants.Lifter.PID_THRESHOLD)){
            currentTime = System.nanoTime();
            currentDistance = lift.getCurrentPosition();
            double dt = (currentTime - previousTime) / 1000000; // In seconds
            power = pid.update(targetDistance,currentDistance,dt);
            lift.setPower(power);
            secondsElapsed += dt;
            previousTime = currentTime;
        }
    }*/
    public void setPower(double power) {
        extender.setPower(power);
    }
    public void setDumpState(DumpState state){
        dumpState = state;
    }
    public String toString(){
        return "" + extender.getCurrentPosition();
    }

}
