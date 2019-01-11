package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Pid;

@TeleOp
public class PidTest extends OpMode {

    private DcMotor lift;
    private Pid pid;
    private AdjustMode mode = AdjustMode.P;
    private boolean xPressed = false;
    private boolean yPressed = false;
    private boolean bPressed = false;
    private boolean aPressed = false;
    private double secondsElapsed = 0;
    private double maxPower = 0.75;
    private double kp = 1.0;
    private double ti = 0.0;
    private double td = 0.0;
    public enum AdjustMode{
        P,I,D
    }

    @Override
    public void init(){
        lift = hardwareMap.dcMotor.get(Constants.Lifter.LIFT);
    }

    @Override
    public void loop(){
        if(gamepad1.dpad_up){
            mode = AdjustMode.P;
        } else if(gamepad1.dpad_right){
            mode = AdjustMode.I;
        } else if(gamepad1.dpad_down){
            mode = AdjustMode.D;
        } else if(gamepad1.dpad_left){
            enableAndResetEncoders();
        }

        if(gamepad1.x && !xPressed){
            switch(mode){
                case P:
                    kp += 0.01;
                    break;
                case I:
                    ti += 0.01;
                    break;
                case D:
                    td += 0.01;
                    break;
            }
            xPressed = true;
        }
        if(gamepad1.y){
            switch(mode){
                case P:
                    kp -= 0.01;
                    break;
                case I:
                    ti -= 0.01;
                    break;
                case D:
                    td -= 0.01;
                    break;
            }
        }
        if(gamepad1.b){
            switch(mode){
                case P:
                    kp += 0.001;
                    break;
                case I:
                    ti += 0.001;
                    break;
                case D:
                    td += 0.001;
                    break;
            }
        }
        if(gamepad1.a){
            switch(mode){
                case P:
                    kp -= 0.001;
                    break;
                case I:
                    ti -= 0.001;
                    break;
                case D:
                    td -= 0.001;
                    break;
            }
        }
    }

    public void enableAndResetEncoders(){
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setPosition(int targetDistance){
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
    }

}
