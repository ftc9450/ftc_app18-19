package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.sensors.Gyroscope;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
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
    private Climber climb;
    private final float initialAngle = -45;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(hardwareMap.dcMotor.get(Constants.Drivetrain.LF), hardwareMap.dcMotor.get(Constants.Drivetrain.LB), hardwareMap.dcMotor.get(Constants.Drivetrain.RF), hardwareMap.dcMotor.get(Constants.Drivetrain.RB));
        imu = new Gyroscope(hardwareMap.get(BNO055IMU.class, "imu"));
        tracker = new MotionTracker(hardwareMap.dcMotor.get(Constants.MotionTracker.FB), hardwareMap.dcMotor.get(Constants.MotionTracker.LR), drivetrain, imu, initialAngle); //TODO: check angle
        climb = new Climber(hardwareMap.dcMotor.get(Constants.Climber.CL));

        drivetrain.enableAndResetEncoders();
        float angleWhenHanging = tracker.getAbsoluteAngle();
        waitForStart();
        // lower robot
        climb.setClimberState(Climber.ClimberState.UP);
        climb.loop();
        Thread.sleep(5000);
        climb.setClimberState(Climber.ClimberState.OFF);
        climb.loop();
        tracker.enableAndResetEncoders();
        //TODO
        //TODO: CHECK ALL THE VALUES I WILL DIE
        //TODO
        drivetrain.moveLR(5,0.3,false,tracker);// move out of latch //
        pivotTo(angleWhenHanging);//correct for difference in angle caused by dropping
        drivetrain.moveFB(18,.75,true,tracker);// drive forward until in front of samples
        int mineralPosition = 1; // placeholder // TODO: get position of gold sample (0, 1, 2) -> (left, center, right)
        switch(mineralPosition){
            //pivot to face gold mineral
            case 0:
                pivotClockwise(45);
                break;
            case 1:
                // should be aligned
                break;
            case 2:
                pivotCounterclockwise(45);
                break;
        }
        drivetrain.moveFB(26,.7,true,tracker);// knock off gold mineral
        drivetrain.moveFB(26,.7,false,tracker);// return to position
        pivotTo(135); // Drive will then drive backwards
        drivetrain.moveFB(50,.7,false,tracker);// drive backward to waypoint (on safe auto paths map)
        pivotCounterclockwise(Math.abs(initialAngle)); // turn left so back is facing depot // TODO: check value, right now it is the same as the initial angle
        drivetrain.moveFB(38,.7,false,tracker);// drive backward until depot
        // TODO: deposit team marker (from back of robot)
        // (shouldn't need to) turn left to face crater
        drivetrain.moveFB(80,.7,true,tracker);// drive forward until parked on crater
    }

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

}
