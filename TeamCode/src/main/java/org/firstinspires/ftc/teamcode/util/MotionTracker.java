package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.sensors.Gyroscope;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/**
 * Created by Avi Trost on 12/15/18.
 */

public class MotionTracker {
    public double x;
    public double y;
    private double angle; // In radians for use with Math.sin(), Math.cos().  Gyroscope in Degrees
    private double offsetAngle; // To make x and y orient correctly and with boundaries, in degrees
    private double previousX;
    private double previousY;
    private double xWhileTurning;
    private double yWhileTurning;
    private DcMotor xOmni;
    private DcMotor yOmni;
    private Gyroscope gyro;
    private Drivetrain drivetrain;
    public MotionTracker(DcMotor forwardOmniWheel, DcMotor sidewaysOmniWheel, Drivetrain drivetrain, Gyroscope gyro, double initialAngle){
        this.drivetrain = drivetrain;
        this.gyro = gyro;

        offsetAngle = initialAngle;
        angle = 0;

        xOmni = sidewaysOmniWheel;
        yOmni = forwardOmniWheel;

        enableAndResetEncoders();
        xOmni.setDirection(DcMotor.Direction.FORWARD);
        yOmni.setDirection(DcMotor.Direction.FORWARD);

        x = 0;
        y = 0;
        previousX = 0;
        previousY = 0;
        xWhileTurning = 0;
        yWhileTurning = 0;
    }

    public void updatePosition(){
        if(drivetrain.getState() == Drivetrain.DrivetrainState.Turning){ // Ensures encoder values while rotating
            xWhileTurning += xOmni.getCurrentPosition() - previousX;                              // will have no false impact on position
            yWhileTurning += yOmni.getCurrentPosition() - previousY;
            x = previousX;
            y = previousY;
        } else{
            angle = Math.toRadians(gyro.getAngle() + offsetAngle);
            previousX = x;
            previousY = y;
            double xTraveled = xOmni.getCurrentPosition() - xWhileTurning - previousX;
            double yTraveled = yOmni.getCurrentPosition() - yWhileTurning - previousY;
            double euclideanDistance = Math.sqrt(Math.pow(xTraveled, 2) + Math.pow(yTraveled, 2));
            x += euclideanDistance * Math.cos(angle);
            y += euclideanDistance * Math.sin(angle);
        }
    }
    public void enableAndResetEncoders(){
        xOmni.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yOmni.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xOmni.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        yOmni.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void disconnectEncoders(){
        xOmni.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yOmni.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
