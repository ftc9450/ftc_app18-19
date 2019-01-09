package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Constants;
@TeleOp
public class EncoderTest extends OpMode{
    DcMotor lateral;
    DcMotor straight;
    public void init() {
        lateral=hardwareMap.dcMotor.get(Constants.MotionTracker.LR);
        lateral.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        straight=hardwareMap.dcMotor.get(Constants.MotionTracker.FB);
        lateral.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        telemetry.addData("lateral: ",lateral.getCurrentPosition());
        telemetry.addData("straight: ", straight.getCurrentPosition());
    }
}
