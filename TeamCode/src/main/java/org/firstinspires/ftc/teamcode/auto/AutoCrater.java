package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.sensors.Gyroscope;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.MotionTracker;

/**
 * Created by Avi on 12/31/2018.
 */
@Autonomous
public class AutoCrater extends LinearOpMode {

    private Drivetrain drivetrain;
    private Gyroscope imu;
    private MotionTracker tracker;
    private final float initialAngle = -45;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(hardwareMap.dcMotor.get(Constants.Drivetrain.LF), hardwareMap.dcMotor.get(Constants.Drivetrain.LB), hardwareMap.dcMotor.get(Constants.Drivetrain.RF), hardwareMap.dcMotor.get(Constants.Drivetrain.RB));
        imu = new Gyroscope(hardwareMap.get(BNO055IMU.class, "imu"));
        tracker = new MotionTracker(hardwareMap.dcMotor.get(Constants.MotionTracker.FB), hardwareMap.dcMotor.get(Constants.MotionTracker.LR), drivetrain, imu, initialAngle); //TODO: check angle


        drivetrain.enableAndResetEncoders();
        float angleWhenHanging = tracker.getAbsoluteAngle();
        waitForStart();
        // TODO: lower robot
        tracker.enableAndResetEncoders();
        // TODO: move out of latch
        // TODO: lower climber
        pivotTo(angleWhenHanging, imu, tracker);
        // TODO: correct for difference in angle
        // TODO: drive forward until in front of samples
        int mineralPosition = 1; // placeholder // TODO: get position of gold sample (0, 1, 2) -> (left, center, right)
        switch(mineralPosition){
            case 0:
                // TODO: initial strafe to mineral
                break;
            case 1:
                // TODO: initial strafe to mineral
                break;
            case 2:
                // TODO: initial strafe to mineral
                break;
        }
        // TODO: correction strafe (if needbe, may remove)
        // TODO: knock off gold mineral
        // TODO: return to position
        pivotClockwise(90, imu, tracker); // pivot 90 degrees right
        // TODO: drive backward to waypoint (on safe auto map)
        pivotCounterclockwise(-initialAngle, imu, tracker); // turn left so back is facing depot // TODO: check value, right now it is the same as the initial angle
        // TODO: drive backward until depot
        // TODO: deposit team marker (from back of robot)
        // TODO: turn left to face crater
        // TODO: drive forward until parked on crater
    }

    public void pivotClockwise(double angle, Gyroscope imu, MotionTracker tracker){ // Turn clockwise given degree angle
        drivetrain.setState(Drivetrain.DrivetrainState.Turning);
        float startAngle = imu.getAngle();
        double power = Constants.Auto.PIVOT_POWER;
        double[] driveSignal = new double[]{power,power,-power,-power};
        drivetrain.setPower(driveSignal);
        while(opModeIsActive() && angle - (imu.getAngle() - startAngle + 360) % 360 > Constants.Auto.PIVOT_THRESHOLD); // TODO: check value
        drivetrain.setPower(new double[]{0,0,0,0});
        tracker.updatePosition();
    }
    public void pivotCounterclockwise(double angle, Gyroscope imu, MotionTracker tracker){ // Turn counterclockwise given degree angle
        drivetrain.setState(Drivetrain.DrivetrainState.Turning);
        float startAngle = imu.getAngle();
        double power = Constants.Auto.PIVOT_POWER;
        double[] driveSignal = new double[]{-power,-power,power,power};
        drivetrain.setPower(driveSignal);
        while(opModeIsActive() && angle - (startAngle - imu.getAngle() + 360) % 360 > Constants.Auto.PIVOT_THRESHOLD); // TODO: check value
        drivetrain.setPower(new double[]{0,0,0,0});
        tracker.updatePosition();
    }
    public void pivotTo(float targetAngle, Gyroscope imu, MotionTracker tracker){ // relative to field boundaries
        float currentAngle = tracker.getAbsoluteAngle();
        if(Math.abs(currentAngle - targetAngle) > 180){ // must cross the line theta = 0
            if(currentAngle > targetAngle){
                pivotClockwise(360 - (currentAngle - targetAngle), imu, tracker);
            } else if(targetAngle > currentAngle){
                pivotCounterclockwise(360 - (targetAngle - currentAngle), imu, tracker);
            }
        } else{
            if(currentAngle > targetAngle){
                pivotCounterclockwise(currentAngle - targetAngle, imu, tracker);
            } else if(targetAngle > currentAngle){
                pivotClockwise(targetAngle - currentAngle, imu, tracker);
            }
        }
    }

}
