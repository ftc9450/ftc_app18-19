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

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(hardwareMap.dcMotor.get(Constants.Drivetrain.LF), hardwareMap.dcMotor.get(Constants.Drivetrain.LB), hardwareMap.dcMotor.get(Constants.Drivetrain.RF), hardwareMap.dcMotor.get(Constants.Drivetrain.RB));
        imu = new Gyroscope(hardwareMap.get(BNO055IMU.class, "imu"));
        tracker = new MotionTracker(hardwareMap.dcMotor.get(Constants.MotionTracker.FB), hardwareMap.dcMotor.get(Constants.MotionTracker.LR), drivetrain, imu, -45); //TODO: check angle


        drivetrain.enableAndResetEncoders();
        float angleWhenHanging = imu.getAngle();
        waitForStart();
        // TODO: lower robot
        tracker.enableAndResetEncoders();
        // TODO: move out of latch
        // TODO: lower climber
        float currentAngle = imu.getAngle();
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
        // TODO: pivot 90 degrees right
        // TODO: drive backward to waypoint (on safe auto map)
        // TODO: turn left so back is facing depot
        // TODO: drive backward until depot
        // TODO: deposit team marker (from back of robot)
        // TODO: turn left to face crater
        // TODO: drive forward until parked on crater
    }

}
