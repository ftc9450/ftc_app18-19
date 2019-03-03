package org.firstinspires.ftc.team9450.tests.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.team9450.util.Constants;

@TeleOp
public class IntakeTest extends OpMode {
    private DcMotor elevator;

    @Override
    public void init() {
        elevator = hardwareMap.dcMotor.get(Constants.Intake.SL);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) elevator.setPower(0.1);
        else if (gamepad1.dpad_down) elevator.setPower(-0.1);
        else elevator.setPower(0);

        telemetry.addData("elevator", elevator.getCurrentPosition());
        telemetry.update();
        // max 1181
    }
}
