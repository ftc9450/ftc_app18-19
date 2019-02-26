package org.firstinspires.ftc.team9450.tests.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team9450.sensors.Gyroscope;
import org.firstinspires.ftc.team9450.subsystems.Drivetrain;
import org.firstinspires.ftc.team9450.util.Constants;
import org.firstinspires.ftc.team9450.util.MotionTracker;

@Autonomous(name = "Straight Path", group = "Auto")
public class StraightPath extends LinearOpMode {
    private Drivetrain drive;
    private Gyroscope imu;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drivetrain(hardwareMap);
        imu = new Gyroscope(hardwareMap.get(BNO055IMU.class, "imu"));
        double TARGET = 1440;
        double error = TARGET;
        double correction = imu.getAngle()/10;
        double power;

        double maxPower = 0;
        double maxError = 0;
        double maxCorrection = 0;

        waitForStart();
        while (opModeIsActive() && error > 0) {
            error = TARGET - drive.getPosition();
            power = 0.3;//0.7 / (1.0 + Math.exp(-error/(TARGET) + 4));
            correction = imu.getAngle()/100;
            drive.setPower(new double[]{power + correction, power + correction, power - correction, power - correction});
            telemetry.addData("power", power);
            telemetry.addData("error", error);
            telemetry.addData("correction", correction);
            if (power > maxPower) maxPower = power;
            if (error > maxError) maxError = error;
            if (correction > maxCorrection) maxCorrection = correction;
            telemetry.update();
        }
        drive.setPower(new double[]{0,0,0,0});
        while(opModeIsActive()){}
    }
}
