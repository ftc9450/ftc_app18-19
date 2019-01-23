package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Constants;
@TeleOp
public class ServoTest extends OpMode{
    Servo hook;
    Servo pawl;
    @Override
    public void init() {
        hook=hardwareMap.servo.get(Constants.Climber.HK);
        hook.setDirection(Servo.Direction.FORWARD);
        pawl=hardwareMap.servo.get(Constants.Climber.PL);
        pawl.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void loop() {
        telemetry.addData("docu","a=hook+,b=hook-,x=pawl+,y=pawl-");
        telemetry.addData("hook",hook.getPosition());
        telemetry.addData("pawl", pawl.getPosition());
        if(gamepad1.a){
            //hook.setPosition(hook.getPosition()+0.01);
            hook.setPosition(Constants.Climber.HOOK_CLOSED);
        }
        if(gamepad1.b){
            //hook.setPosition(hook.getPosition()-0.01);
            hook.setPosition(Constants.Climber.HOOK_OPEN);
        }
        if(gamepad1.x){
            pawl.setPosition(pawl.getPosition()+0.01);
        }
        if(gamepad1.y){
            pawl.setPosition(pawl.getPosition()-0.01);
        }
    }
}
