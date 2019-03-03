package org.firstinspires.ftc.team9450.util;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.apache.commons.math3.analysis.function.Gaussian;
import org.firstinspires.ftc.team9450.sensors.Gyroscope;
import org.firstinspires.ftc.team9450.subsystems.Drivetrain;
import org.firstinspires.ftc.team9450.subsystems.Subsystem;

public class MotionPlanner {
    private Drivetrain drive;
    private Gyroscope imu;
    double init;
    double TARGET;
    double errorCorrect;
    double error;
    double correction;
    double power;
    Gaussian gauss;
    boolean a;

    public MotionPlanner(Drivetrain dt, Gyroscope gyro) {
        drive = dt;
        imu = gyro;
        init = drive.getPosition();
    }

    public void moveFB(double distance) {
        TARGET = distance;
        errorCorrect= 1174*(Math.pow(0.9997, TARGET));
        error = TARGET;
        correction = imu.getAngle()/100;
        gauss = new Gaussian(TARGET/2.0, TARGET/6.0);
        a = TARGET > errorCorrect;
        if (a) TARGET -= errorCorrect;
        error = TARGET;
    }

    public double loop() {
        if (a && error > 0) {
            error = TARGET - drive.getPosition() - init;
            power = (500*gauss.value(error)) + 0.15;
            correction = imu.getAngle()/100;
            drive.setPower(new double[]{power + correction, power + correction, power - correction, power - correction});
        } else if (error > 0) {
            error = TARGET - drive.getPosition() - init;
            power = 0.2;
            correction = imu.getAngle()/100;
            drive.setPower(new double[]{power + correction, power + correction, power - correction, power - correction});
        }
        return error;
    }
}
