package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.sensors.Gyroscope;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.MotionTracker;
import org.firstinspires.ftc.teamcode.util.DetectCube;
import org.opencv.core.*;
import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.view.View.OnTouchListener;
import android.view.SurfaceView;
public class AutoDepot extends LinearOpMode {
    private Drivetrain drivetrain;
    private Gyroscope imu;
    private MotionTracker tracker;
    Climber climb;
    public void runOpMode() throws InterruptedException {
        waitForStart();
        drivetrain = new Drivetrain(hardwareMap.dcMotor.get(Constants.Drivetrain.LF), hardwareMap.dcMotor.get(Constants.Drivetrain.LB), hardwareMap.dcMotor.get(Constants.Drivetrain.RF), hardwareMap.dcMotor.get(Constants.Drivetrain.RB));
        imu = new Gyroscope(hardwareMap.get(BNO055IMU.class, "imu"));
        tracker = new MotionTracker(hardwareMap.dcMotor.get(Constants.MotionTracker.FB), hardwareMap.dcMotor.get(Constants.MotionTracker.LR), drivetrain);
        drivetrain.enableAndResetEncoders();
        climb = new Climber(hardwareMap.dcMotor.get(Constants.Climber.CL));

        float angleWhenHanging = imu.getAngle();
        // TODO: drop
        float currentAngle = imu.getAngle();
        // TODO: correct for difference in angle
        tracker.enableAndResetEncoders();
        // TODO: drive forward until in front of samples
        // TODO:
    }
    //Straffe method to straffe based on input from vision
    public void cubemove() throws InterruptedException{
        int x = DetectCube.process(Mat source1);


    }
}
