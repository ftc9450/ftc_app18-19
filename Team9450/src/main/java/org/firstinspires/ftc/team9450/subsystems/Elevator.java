package org.firstinspires.ftc.team9450.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team9450.util.Constants;


public class Elevator extends Subsystem {

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
        IN, OUT
    }
    public Elevator(HardwareMap map){
        extender = map.dcMotor.get(Constants.Elevator.ELEVATOR);
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extender.setDirection(DcMotor.Direction.FORWARD);
        dump = map.servo.get(Constants.Elevator.DUMP);
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
            case IN:
                dump.setPosition(Constants.Elevator.DUMP_OPEN);
                break;
            case OUT:
                dump.setPosition(Constants.Elevator.DUMP_CLOSED);
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
        while(Math.abs(currentDistance) < Math.abs(targetDistance - Constants.Elevator.PID_THRESHOLD)){
            currentTime = System.nanoTime();
            currentDistance = extender.getCurrentPosition();
            double dt = (currentTime - previousTime) / 1000000; // In seconds
            secondsElapsed += dt;
            previousTime = currentTime;
        }
    }
    public void setExtendState(ExtendState state){
        extendState = state;
        switch(extendState){
            case UP:
                extender.setTargetPosition(Constants.Elevator.UP_POSITION);
                break;
            case DOWN:
                extender.setPower(Constants.Elevator.DOWN_POSITION);
                break;
        }
    }
    public void setDumpState(DumpState state){
        dumpState = state;
    }
    public String toString(){
        return "" + extender.getCurrentPosition();
    }

}
