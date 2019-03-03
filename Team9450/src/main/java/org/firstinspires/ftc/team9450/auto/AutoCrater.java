package org.firstinspires.ftc.team9450.auto;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.team9450.sensors.Gyroscope;
import org.firstinspires.ftc.team9450.subsystems.Climber;
import org.firstinspires.ftc.team9450.subsystems.Drivetrain;
import org.firstinspires.ftc.team9450.util.Constants;
import org.firstinspires.ftc.team9450.util.MotionTracker;
import org.opencv.core.Mat;


/**
 * Created by Avi on 12/31/2018.
 */
@Autonomous
public class AutoCrater extends LinearOpMode {

    private Drivetrain drivetrain;
    private Gyroscope imu;
    private MotionTracker tracker;
    private Climber climb;
    private GoldAlignDetector detector;
    private final float initialAngle = -45;

    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain = new Drivetrain(hardwareMap.dcMotor.get(Constants.Drivetrain.LF), hardwareMap.dcMotor.get(Constants.Drivetrain.LB),
                hardwareMap.dcMotor.get(Constants.Drivetrain.RF), hardwareMap.dcMotor.get(Constants.Drivetrain.RB));

        imu = new Gyroscope(hardwareMap.get(BNO055IMU.class, "imu"));
        tracker = new MotionTracker(hardwareMap.dcMotor.get(Constants.MotionTracker.FB), hardwareMap.dcMotor.get(Constants.MotionTracker.LR), drivetrain, imu, initialAngle); //TODO: check angle
        climb = new Climber(hardwareMap);
        drivetrain.enableAndResetEncoders();
        climb.setElevatorState(Climber.ElevatorState.DOWN);
        drivetrain.moveFB(4,.5,true,tracker);
        pivotTo(90);
        drivetrain.moveFB(15,.5,true,tracker);
        pivotTo(-90);
        drivetrain.moveFB(45,.5,true,tracker);
        pivotTo(-45);
        drivetrain.moveFB(40,.5,true,tracker);
        drivetrain.moveFB(40,.5,false,tracker);
        pivotTo(-135);
        drivetrain.moveFB(50,.5,true,tracker);
        pivotTo(-135);
        drivetrain.moveFB(60,.5,true,tracker);
        drivetrain.moveFB(4,.5,true,tracker);
        pivotTo(90);
        drivetrain.moveFB(15,.5,true,tracker);
        pivotTo(-90);
        drivetrain.moveFB(45,.5,true,tracker);
        pivotTo(-45);
        drivetrain.moveFB(50,.5,true,tracker);
        drivetrain.moveFB(50,.5,false,tracker);
        pivotTo(-135);
        drivetrain.moveFB(50,.5,true,tracker);
        pivotTo(-90);
        pivotTo(10);
        drivetrain.moveFB(4,.5,true,tracker);
        pivotTo(90);
        drivetrain.moveFB(15,.5,true,tracker);
        pivotTo(-90);
        drivetrain.moveFB(45,.5,true,tracker);
        pivotTo(-45);
        drivetrain.moveFB(50,.5,true,tracker);
        drivetrain.moveFB(50,.5,false,tracker);
        pivotTo(-135);
        drivetrain.moveFB(50,.5,true,tracker);
        pivotTo(-45);
        pivotTo(-35);
    }
/*
    public void pivotClockwise(double angle){ // Turn clockwise given degree angle
        drivetrain.setState(Drivetrain.DrivetrainState.Turning);
        float startAngle = imu.getAngle();
        double power = Constants.Auto.MAX_PIVOT_POWER;
        double[] driveSignal = new double[]{power,power,-power,-power};
        drivetrain.setPower(driveSignal);
        while(opModeIsActive() && angle - (imu.getAngle() - startAngle + 360) % 360 > Constants.Auto.PIVOT_THRESHOLD); // TODO: check value
        drivetrain.setPower(new double[]{0,0,0,0});
        tracker.updatePosition();
    }
    public void pivotCounterclockwise(double angle){ // Turn counterclockwise given degree angle
        drivetrain.setState(Drivetrain.DrivetrainState.Turning);
        float startAngle = imu.getAngle();
        double power = Constants.Auto.MAX_PIVOT_POWER;
        double[] driveSignal = new double[]{-power,-power,power,power};
        drivetrain.setPower(driveSignal);
        while(opModeIsActive() && angle - (startAngle - imu.getAngle() + 360) % 360 > Constants.Auto.PIVOT_THRESHOLD); // TODO: check value
        drivetrain.setPower(new double[]{0,0,0,0});
        tracker.updatePosition();
    }

    public void pivotTo(float targetAngle){ // relative to field boundaries
        float currentAngle = tracker.getAbsoluteAngle();
        if(Math.abs(currentAngle - targetAngle) > 180){ // must cross the line theta = 0
            if(currentAngle > targetAngle){
                pivotClockwise(360 - (currentAngle - targetAngle));
            } else if(targetAngle > currentAngle){
                pivotCounterclockwise(360 - (targetAngle - currentAngle));
            }
        } else{
            if(currentAngle > targetAngle){
                pivotCounterclockwise(currentAngle - targetAngle);
            } else if(targetAngle > currentAngle){
                pivotClockwise(targetAngle - currentAngle);
            }
        }
    }*/
    public void pivotTo(float targetAngle){
        double power =0.5;
        imu.zero();
        if(targetAngle>0){//ccwise
            drivetrain.setPower(new double[]{-power, -power, power, power});
        }else{
            drivetrain.setPower(new double[]{power, power, -power, -power});
        }
        while(opModeIsActive() && Math.abs(imu.getAngle() - targetAngle) > Math.PI/10){}
        drivetrain.setPower(new double[]{0, 0, 0, 0});
    }
}