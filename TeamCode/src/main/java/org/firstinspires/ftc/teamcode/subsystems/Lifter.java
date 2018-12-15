package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class Lifter extends Subsystem {

    private DcMotor lift;
    double power = 0;
    private double maxPower;
    private double[] driveSignal;
    public void lifter(DcMotor l){
        lift = l;
    }
    public void setPower(double power) {
        lift.setPower(power);
    }
    public void setPower(double driveSignal[]) {
        lift.setPower(driveSignal[0]);
    }
    public void loop() {
        setPower(driveSignal);
    }
    public void stop() {
        lift.setPower(0);
    }

}
