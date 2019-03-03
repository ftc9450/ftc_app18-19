package org.firstinspires.ftc.team9450.util;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.apache.commons.math3.analysis.function.Gaussian;
import org.firstinspires.ftc.team9450.sensors.Gyroscope;
import org.firstinspires.ftc.team9450.subsystems.Drivetrain;

public class MotionPlanner {
    private Drivetrain drive;
    private Gyroscope imu;
    private double init;
    private double TARGET;
    private double errorCorrect;
    private double error;
    private double correction;
    private double power;
    private Gaussian gauss;
    private boolean a;
    private State state;


    public MotionPlanner(Drivetrain dt, Gyroscope gyro) {
        drive = dt;
        imu = gyro;
    }

    public enum State {
        LINEAR, TURNING
    }

    public void moveFB(double distance) {
        state = State.LINEAR;
        init = drive.getPosition();
        TARGET = distance;
        errorCorrect= 1174*(Math.pow(0.9997, TARGET));
        error = TARGET;
        correction = imu.getAngle()/100;
        gauss = new Gaussian(TARGET/2.0, TARGET/6.0);
        a = TARGET > errorCorrect;
        if (a) TARGET -= errorCorrect;
        error = TARGET;
    }

    public void pivot(double angle) {
        state = State.TURNING;
        TARGET = angle;
        imu.zero();
    }

    public double loop() {
        switch (state) {
            case LINEAR:
                if (a && error > 0) {
                    error = TARGET - drive.getPosition() + init;
                    power = (500 * gauss.value(error)) + 0.15;
                    correction = imu.getAngle() / 100;
                    drive.setPower(new double[]{power + correction, power + correction, power - correction, power - correction});
                } else if (error > 0) {
                    error = TARGET - drive.getPosition() + init;
                    power = 0.2;
                    correction = imu.getAngle() / 100;
                    drive.setPower(new double[]{power + correction, power + correction, power - correction, power - correction});
                } else {
                    drive.setPower(0);
                    return 0;
                }
                break;

            case TURNING:
                if (error > 0 && TARGET > 0) {
                    error = TARGET - imu.getAngle();
                    power = Constants.Drivetrain.PIVOT_POWER;
                    drive.setPower(new double[]{-power, -power, power, power});
                } else if (error > 0 && TARGET > 0) {
                    error = imu.getAngle() - TARGET;
                    power = Constants.Drivetrain.PIVOT_POWER;
                    drive.setPower(new double[]{power, power, -power, -power});
                } else {
                    drive.setPower(0);
                    return 0;
                }
                break;
        }
        return error;
    }
}
