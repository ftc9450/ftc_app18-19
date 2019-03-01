package org.firstinspires.ftc.team9450.tests.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class climbtest extends OpMode{

    DcMotor climber;

    public void init(){
        climber = hardwareMap.get(DcMotor.class, "climber");
        climber.setDirection(DcMotor.Direction.FORWARD);
        climber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop(){
        if(gamepad1.dpad_up){
            climber.setPower(0.3);
        }else if(gamepad1.dpad_down){
            climber.setPower(-0.3);
        }
        telemetry.addData("Position", climber.getCurrentPosition());
        telemetry.update();

    }

}