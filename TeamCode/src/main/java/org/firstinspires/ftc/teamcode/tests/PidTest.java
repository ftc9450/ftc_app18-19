package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.sensors.Gyroscope;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.MotionTracker;
import org.firstinspires.ftc.teamcode.util.Pid;

@TeleOp
public class PidTest extends OpMode {

    private DcMotor lift;
    private Drivetrain drivetrain;
    private MotionTracker tracker;
    private Gyroscope imu;
    private Pid pid;
    private AdjustMode mode = AdjustMode.P;
    private boolean xPressed = false;
    private boolean yPressed = false;
    private boolean bPressed = false;
    private boolean aPressed = false;
    private boolean rbPressed = false;
    private boolean lbPressed = false;
    private double secondsElapsed = 0;
    private double Tu = 0;
    private boolean firstOvershoot = true;
    private boolean inOvershoot = false;
    private boolean firstDip = false;
    private double maxPower = 0.75;
    private double kp = 0.1;
    private double ti = 0.0;
    private double td = 0.0;
    public enum AdjustMode{
        P,I,D
    }

    @Override
    public void init(){
        imu = new Gyroscope(hardwareMap.get(BNO055IMU.class, "imu"));
        drivetrain = new Drivetrain(hardwareMap.dcMotor.get(Constants.Drivetrain.LF),hardwareMap.dcMotor.get(Constants.Drivetrain.LB), hardwareMap.dcMotor.get(Constants.Drivetrain.RF), hardwareMap.dcMotor.get(Constants.Drivetrain.RB));
        tracker = new MotionTracker(hardwareMap.dcMotor.get(Constants.MotionTracker.FB), hardwareMap.dcMotor.get(Constants.MotionTracker.LR), drivetrain, imu, 0);

        //lift = hardwareMap.dcMotor.get(Constants.Lifter.LIFT);
    }

    @Override
    public void loop(){
        if(gamepad1.dpad_up){
            mode = AdjustMode.P;
        } else if(gamepad1.dpad_left){
            mode = AdjustMode.I;
        } else if(gamepad1.dpad_right){
            mode = AdjustMode.D;
        } else if(gamepad1.dpad_down){
            enableAndResetEncoders();
        }

        if(gamepad1.x){
            if(!xPressed){
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
            }
            xPressed = true;
        } else{
            xPressed = false;
        }
        if(gamepad1.y){
            if(!yPressed) {
                switch (mode) {
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
                yPressed = true;
            }
        } else{
            yPressed = false;
        }
        if(gamepad1.b){
            if(!bPressed) {
                switch (mode) {
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
                bPressed = true;
            }
        } else{
            bPressed = false;
        }
        if(gamepad1.a){
            if(!aPressed) {
                switch (mode) {
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
                aPressed = true;
            }
        } else{
            aPressed = false;
        }
        if(gamepad1.right_bumper){
            if(!rbPressed) {
                setPosition(3000);
                rbPressed = true;
            }
        } else{
            rbPressed = false;
        }

        telemetry.addData("Mode:", mode);
        telemetry.addData("P:", kp);
        telemetry.addData("I:", ti);
        telemetry.addData("D:", td);
        telemetry.addData("Tu:", Tu);
        telemetry.update();

    }

    public void enableAndResetEncoders(){
        tracker.enableAndResetEncoders();
        /*
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */
    }

    private void setPosition(int distance){
        enableAndResetEncoders();
        double targetDistance = distance;// * Constants.MotionTracker.CLICKS_PER_INCH;
        pid = new Pid(kp,ti,td,-1,1,-maxPower,maxPower);
        double power;
        int currentDistance = 0;
        secondsElapsed = 0;
        double previousTime = System.nanoTime();
        double currentTime;
        boolean overshoot = false;
        while(true){ // to test overshoot amount
        //while(Math.abs(currentDistance) < Math.abs(targetDistance - Constants.Drivetrain.FB_PID_THRESHOLD)){
            currentTime = System.nanoTime();
            currentDistance = tracker.getYEncoderValue();
            double dt = (currentTime - previousTime) / 1000000; // In seconds
            double error = currentDistance - targetDistance;
            if(firstOvershoot){ // get period of oscillation
                if(error > 0) {
                    if(firstDip){
                        firstOvershoot = false;
                    }
                    inOvershoot = true;
                    Tu += dt;
                }else if(inOvershoot){
                    Tu += dt;
                    firstDip = true;
                }
            }
            power = pid.update(targetDistance,currentDistance,dt);
            double[] signal = new double[]{power*Constants.Drivetrain.FB_LEFT_POWER,power*Constants.Drivetrain.FB_LEFT_POWER,power*Constants.Drivetrain.FB_RIGHT_POWER,power*Constants.Drivetrain.FB_RIGHT_POWER};
            drivetrain.setPower(signal);
            secondsElapsed += dt;
            previousTime = currentTime;
            tracker.updatePosition();
            telemetry.update();
        }
    }

    /*
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
    */

}
