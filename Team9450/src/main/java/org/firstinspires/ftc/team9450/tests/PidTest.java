package org.firstinspires.ftc.team9450.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.team9450.util.Pid;

@TeleOp
public class PIDTest extends OpMode {
    DcMotor motor;

    public static final double P = 2.5;
    public static final double I = 0.1;
    public static final double D = 0.2;

    PIDFCoefficients pidOrig;
    PIDFCoefficients pidModified;

    public void init() {
        motor = hardwareMap.get(DcMotor.class, "left_drive");


        // get a reference to the motor controller and cast it as an extended functionality controller.
        // we assume it's a REV Robotics Expansion Hub (which supports the extended controller functions).
        DcMotorControllerEx motorControllerEx = (DcMotorControllerEx) motor.getController();

        // get the port number of our configured motor.
        int motorIndex = ((DcMotorEx) motor).getPortNumber();

        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        pidOrig = motorControllerEx.getPIDFCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER);

        // change coefficients.
        PIDFCoefficients pidNew = new PIDFCoefficients(P, I, D, 0);
        motorControllerEx.setPIDFCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        // re-read coefficients and verify change.
        pidModified = motorControllerEx.getPIDFCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void loop() {
        telemetry.addData("Runtime", "%.03f", getRuntime());
        telemetry.addData("P,I,D (orig)", "%.04f, %.04f, %.0f",
                pidOrig.p, pidOrig.i, pidOrig.d);
        telemetry.addData("P,I,D (modified)", "%.04f, %.04f, %.04f",
                pidModified.p, pidModified.i, pidModified.d);
        telemetry.update();
    }
}
