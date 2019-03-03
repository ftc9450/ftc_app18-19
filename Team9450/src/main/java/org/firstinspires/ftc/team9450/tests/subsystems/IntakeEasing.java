package org.firstinspires.ftc.team9450.tests.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.apache.commons.math3.analysis.function.Gaussian;

public class IntakeEasing extends OpMode {
    private DcMotor intake;
    double err;
    double LIMIT = 1181;
    Gaussian gauss = new Gaussian(LIMIT/2.0, LIMIT/6.0);

    @Override
    public void init() {
        intake = hardwareMap.dcMotor.get("slider");
        err = LIMIT;
    }

    @Override
    public void loop() {
        err = LIMIT - intake.getCurrentPosition();
        //intake.setPower(gauss);
    }
}
