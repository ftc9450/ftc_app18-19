package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class Lifter extends Subsystem {

    DcMotor lift;
    double power = 0;

    public void lifter(){

        lift = hardwareMap.dcMotor.get("lift");

        waitForstart();

        while (opModeIsActive()) {

            lift.setPower(gamepad1.left_stick_y);
            telemetry.addData("joystick", gamepad1.a);

        }
        stop();
    }
}
