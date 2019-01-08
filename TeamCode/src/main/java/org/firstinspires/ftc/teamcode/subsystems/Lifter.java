package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Pid;

@TeleOp

public class Lifter extends Subsystem {

    private DcMotor lift;
    double power = 1;
    private double maxPower = 0.75;
    public String state = "off";
    private LifterState lifterState;
    private Pid pid = null;
    private final double kp = 1.0;
    private final double ti = 0.0;
    private final double td = 0.0;
    public enum LifterState{
        UP, DOWN
    }
    public Lifter(DcMotor li){
        lift = li;
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setDirection(DcMotor.Direction.FORWARD); // TODO: check direction
    }

    public void enableAndResetEncoders() {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void loop() {
        switch(lifterState){
            case UP:
                setPosition(Constants.Lifter.UP_POSITION);
                break;
            case DOWN:
                setPosition(-Constants.Lifter.UP_POSITION);
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
        pid = new Pid(kp,ti,td,-1,1,-maxPower,maxPower);
        double power;
        int currentDistance = 0;
        double secondsElapsed = 0;
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
    }
    public void setLifterState(LifterState state){
        lifterState = state;
    }

}
