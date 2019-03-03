package org.firstinspires.ftc.team9450.tests.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Autonomous
public class ClimberPID extends LinearOpMode {

    // our DC motor.
    DcMotorEx climber;

    public static final double NEW_P = 2.5;
    public static final double NEW_I = 0.1;
    public static final double NEW_D = 0.2;

    public void runOpMode() {
        // get reference to DC motor.
        // since we are using the Expansion Hub,
        // cast this motor to a DcMotorEx object.
        climber = (DcMotorEx)hardwareMap.get(DcMotor.class, "climber");

        // wait for start command.
        waitForStart();

        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDFCoefficients pidOrig = climber.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, 0);
        climber.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        // re-read coefficients and verify change.
        PIDFCoefficients pidModified = climber.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // display info to user.
        while(opModeIsActive()) {
            if(gamepad1.x){
                climber.setTargetPosition(1558); //TODO: check
                climber.setPower(0.75);
            }else if(gamepad1.y){
                climber.setTargetPosition(200); //TODO: check
                climber.setPower(0.75);
            }

            telemetry.addData("Position", climber.getCurrentPosition());
            telemetry.addData("Runtime", "%.03f", getRuntime());
            telemetry.addData("P,I,D (orig)", "%.04f, %.04f, %.0f",
                    pidOrig.p, pidOrig.i, pidOrig.d);
            telemetry.addData("P,I,D (modified)", "%.04f, %.04f, %.04f",
                    pidModified.p, pidModified.i, pidModified.d);
            telemetry.update();
        }
    }
}
