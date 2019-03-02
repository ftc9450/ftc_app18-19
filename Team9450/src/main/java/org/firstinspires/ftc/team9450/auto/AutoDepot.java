package org.firstinspires.ftc.team9450.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team9450.sensors.Gyroscope;
import org.firstinspires.ftc.team9450.subsystems.Climber;
import org.firstinspires.ftc.team9450.subsystems.Drivetrain;
import org.firstinspires.ftc.team9450.util.Constants;
import org.firstinspires.ftc.team9450.util.MotionTracker;


/**
 * Created by Avi on 12/31/2018.
 */
@Autonomous
public class AutoDepot extends LinearOpMode {

    private Drivetrain drivetrain;
    private Gyroscope imu;
    private MotionTracker tracker;
    private Climber climb;
    private final float initialAngle = 45;

    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain = new Drivetrain(hardwareMap.dcMotor.get(Constants.Drivetrain.LF), hardwareMap.dcMotor.get(Constants.Drivetrain.LB), hardwareMap.dcMotor.get(Constants.Drivetrain.RF), hardwareMap.dcMotor.get(Constants.Drivetrain.RB));

        //imu = new Gyroscope(hardwareMap.get(BNO055IMU.class, "imu"));
        //tracker = new MotionTracker(hardwareMap.dcMotor.get(Constants.MotionTracker.FB), hardwareMap.dcMotor.get(Constants.MotionTracker.LR), drivetrain, imu, initialAngle); //TODO: check angle
        tracker = new MotionTracker(hardwareMap.dcMotor.get(Constants.MotionTracker.FB), hardwareMap.dcMotor.get(Constants.MotionTracker.LR), drivetrain, initialAngle); //TODO: check angle

        /*climb = new Climber(hardwareMap.dcMotor.get(Constants.Climber.EL),hardwareMap.dcMotor.get(Constants.Climber.PI),
                hardwareMap.servo.get(Constants.Climber.HK), hardwareMap.servo.get(Constants.Climber.PL));*/
        drivetrain.enableAndResetEncoders();
        climb.setElevatorState(Climber.ElevatorState.DOWN);
        drivetrain.moveFB(4,.5,true,tracker);
        pivotTo(90);
        drivetrain.moveFB(15,.5,true,tracker);
        pivotTo(100);
        drivetrain.moveFB(40,.5,false,tracker);
        pivotTo(-55);
        drivetrain.moveFB(50,.5,true,tracker);
        drivetrain.moveFB(55,.5,false,tracker);//dropping off marker
        pivotTo(45);
        drivetrain.moveFB(50,.5,true,tracker);
        pivotTo(-135);
        pivotTo(55);
        pivotTo(-100);
        drivetrain.moveFB(50,.5,true,tracker);
        pivotTo(-35);
        drivetrain.moveFB(10,.5,true,tracker);
        drivetrain.moveFB(4,.5,true,tracker);
        pivotTo(90);
        drivetrain.moveFB(15,.5,true,tracker);
        pivotTo(100);
        drivetrain.moveFB(40,.5,false,tracker);
        pivotTo(-55);
        drivetrain.moveFB(50,.5,true,tracker);
        drivetrain.moveFB(55,.5,false,tracker);
        pivotTo(45);
        drivetrain.moveFB(50,.5,true,tracker);
        pivotTo(-90);
        wait(500);
        pivotTo(90);
        pivotTo(-100);
        drivetrain.moveFB(50,.5,true,tracker);
        pivotTo(-35);
        drivetrain.moveFB(10,.5,true,tracker);
        drivetrain.moveFB(4,.5,true,tracker);
        pivotTo(90);
        drivetrain.moveFB(15,.5,true,tracker);
        pivotTo(100);
        drivetrain.moveFB(40,.5,false,tracker);
        pivotTo(-55);
        drivetrain.moveFB(50,.5,true,tracker);
        drivetrain.moveFB(55,.5,false,tracker);
        pivotTo(45);
        drivetrain.moveFB(50,.5,true,tracker);
        pivotTo(-45);
        wait(500);
        pivotTo(-45);
        wait(500);
        pivotTo(90);
        drivetrain.moveFB(50,.5,true,tracker);
        pivotTo(-35);
        drivetrain.moveFB(10,.5,true,tracker);
    }
    public void pivotClockwise(int angle){
        drivetrain.setPower(new double[]{0.05,0.05,-0.05,-0.05});
        sleep(5*angle);
        drivetrain.setPower(new double[]{0,0,0,0});
    }
    public void pivotCounterclockwise(int angle){
        drivetrain.setPower(new double[]{-0.05,-0.05,0.05,0.05});
        sleep(5*angle);
        drivetrain.setPower(new double[]{0,0,0,0});
    }
    public void pivot(double angle, boolean cc) {
        double Q = Math.PI/25;
        imu = new Gyroscope(hardwareMap.get(BNO055IMU.class, "imu"));
        if (cc) {
            drivetrain.setPower(new double[]{-0.3, -0.3, 0.3, 0.3});
        } else {
            drivetrain.setPower(new double[]{0.3, 0.3, -0.3, -0.3});
        }
        while (opModeIsActive() && Math.abs(imu.getAngle()) < angle - Q) {}
        drivetrain.setPower(0);
    }
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

/*
    public void pivotClockwise(double angle){ // Turn clockwise given degree angle
        drivetrain.setState(Drivetrain.DrivetrainState.Turning);
        float startAngle = imu.getAngle();
        double power = Constants.Auto.PIVOT_POWER;
        double[] driveSignal = new double[]{power,power,-power,-power};
        drivetrain.setPower(driveSignal);
        while(opModeIsActive() && angle - (imu.getAngle() - startAngle + 360) % 360 > Constants.Auto.PIVOT_THRESHOLD); // TODO: check value
        drivetrain.setPower(new double[]{0,0,0,0});
        tracker.updatePosition();
    }
    
    public void pivotCounterclockwise(double angle){ // Turn counterclockwise given degree angle
        drivetrain.setState(Drivetrain.DrivetrainState.Turning);
        float startAngle = imu.getAngle();
        double power = Constants.Auto.PIVOT_POWER;
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
    }
*/
}

