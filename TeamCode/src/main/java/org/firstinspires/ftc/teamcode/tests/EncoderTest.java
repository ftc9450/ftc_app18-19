package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.sensors.Gyroscope;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.MotionTracker;

@TeleOp
public class EncoderTest extends OpMode{
    DcMotor lateral;
    DcMotor straight;
    Gyroscope imu;
    Drivetrain drivetrain;
    MotionTracker tracker;
    public void init() {
        imu = new Gyroscope(hardwareMap.get(BNO055IMU.class, "imu"));
        drivetrain = new Drivetrain(hardwareMap.dcMotor.get(Constants.Drivetrain.LF),hardwareMap.dcMotor.get(Constants.Drivetrain.LB), hardwareMap.dcMotor.get(Constants.Drivetrain.RF), hardwareMap.dcMotor.get(Constants.Drivetrain.RB));
        tracker = new MotionTracker(hardwareMap.dcMotor.get(Constants.MotionTracker.FB), hardwareMap.dcMotor.get(Constants.MotionTracker.LR), drivetrain, imu, 0);

        /*
        lateral=hardwareMap.dcMotor.get(Constants.MotionTracker.LR);
        lateral.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        straight=hardwareMap.dcMotor.get(Constants.MotionTracker.FB);
        straight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */
    }

    @Override
    public void loop() {
        telemetry.addData("angle: ", tracker.getAbsoluteAngle());
        telemetry.addData("x: ", tracker.x);
        telemetry.addData("y: ", tracker.y);
        telemetry.addData("lateral: ", tracker.getXEncoderValue());
        telemetry.addData("straight: ", tracker.getYEncoderValue());
        telemetry.update();
    }
}
