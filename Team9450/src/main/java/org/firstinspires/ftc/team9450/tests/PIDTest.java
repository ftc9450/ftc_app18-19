package org.firstinspires.ftc.team9450.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team9450.util.Pid;

public class PIDTest extends OpMode {
    private DcMotor motor;
    private Pid pid;
    @Override
    public void init() {
        motor = hardwareMap.dcMotor.get("motor");
        pid = new Pid(1, 1, 1, 1, 1, 1, 1);
    }

    @Override
    public void loop() {
        pid.update(0, 0, 0);
    }
}
