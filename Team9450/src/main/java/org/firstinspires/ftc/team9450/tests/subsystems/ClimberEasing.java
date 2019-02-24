package org.firstinspires.ftc.team9450.tests.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ClimberEasing extends OpMode {
    private DcMotor motor;

    @Override
    public void init() {
        motor = hardwareMap.dcMotor.get("climber");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            motor.setPower(1);
        } else if (gamepad1.b) {
            motor.setPower(-1);
        } else {
            motor.setPower(0);
        }
    }
}
