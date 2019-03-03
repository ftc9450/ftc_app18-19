package org.firstinspires.ftc.team9450.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team9450.sensors.Gyroscope;
import org.firstinspires.ftc.team9450.subsystems.Climber;
import org.firstinspires.ftc.team9450.subsystems.Drivetrain;
import org.firstinspires.ftc.team9450.subsystems.Intake;
import org.firstinspires.ftc.team9450.util.Constants;
import org.firstinspires.ftc.team9450.util.MotionTracker;


/**
 * Created by Avi on 12/31/2018.
 */
@Autonomous
public class CycleDepot extends LinearOpMode {

    private Drivetrain drivetrain;
    private Gyroscope imu;
    private MotionTracker tracker;
    private Climber climb;
    private Intake intake;

    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain = new Drivetrain(hardwareMap.dcMotor.get(Constants.Drivetrain.LF), hardwareMap.dcMotor.get(Constants.Drivetrain.LB), hardwareMap.dcMotor.get(Constants.Drivetrain.RF), hardwareMap.dcMotor.get(Constants.Drivetrain.RB));

        imu = new Gyroscope(hardwareMap.get(BNO055IMU.class, "imu"));
        //intake = new Intake();
        climb = new Climber(hardwareMap);
        drivetrain.enableAndResetEncoders();
        double power = 0.05;

        waitForStart();
        climb.setElevatorState(Climber.ElevatorState.DOWN);
        climb.loop();
        drivetrain.moveFB(4);
        drivetrain.pivot(-90);
        drivetrain.moveFB(15);
        drivetrain.pivot(90);
        drivetrain.moveFB(45);
        drivetrain.pivot(45);
        drivetrain.moveFB(50);
        drivetrain.moveFB(-50);
        drivetrain.pivot(135);
        drivetrain.moveFB(50);

        int mineralPosition = 1; // placeholder // TODO: get position of gold sample (0, 1, 2) -> (left, center, right)
        switch (mineralPosition) {
            //pivot to face gold mineral
            case 0:
                drivetrain.pivot(135);
                //intake out
                //intake in
                drivetrain.pivot(-60);
                //elevator up and dump
                break;
            case 1:
                drivetrain.pivot(90);
                //intake out
                //intake in
                drivetrain.pivot(-10);
                //elevator up and dump
                break;
            case 2:
                drivetrain.pivot(45);
                //intake out
                //intake in
                drivetrain.pivot(-35);
                //elevator up and dump
                break;
        }
        //intake out
    }
}
