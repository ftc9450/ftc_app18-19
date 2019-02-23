package org.firstinspires.ftc.team9450.tests.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team9450.sensors.Gyroscope;
import org.firstinspires.ftc.team9450.subsystems.Drivetrain;
import org.firstinspires.ftc.team9450.util.Constants;
import org.firstinspires.ftc.team9450.util.MotionTracker;

@Autonomous(name = "Straight Path", group = "test path")
public class StraightPath extends LinearOpMode {
    private Drivetrain drive;
    private Gyroscope imu;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drivetrain(hardwareMap);
        imu = new Gyroscope(hardwareMap.get(BNO055IMU.class, "imu"));
        double TARGET = 10000;
        double error = TARGET;
        double correction = imu.getAngle()/10;
        double power;

        waitForStart();
        while (opModeIsActive()) {
            error = TARGET - drive.getPosition();
            power = 1.0 / (1.0 + Math.exp(-error/2500.0 + 4));
            correction = imu.getAngle()/10;
            drive.setPower(new double[]{power + correction, power + correction, power - correction, power - correction});
        }
    }
}
