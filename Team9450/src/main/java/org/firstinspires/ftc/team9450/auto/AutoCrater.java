package org.firstinspires.ftc.team9450.auto;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
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
        /*climb = new Climber(hardwareMap.dcMotor.get(Constants.Climber.EL), hardwareMap.dcMotor.get(Constants.Climber.PI),
                hardwareMap.servo.get(Constants.Climber.HK),hardwareMap.servo.get(Constants.Climber.PL));*/
        drivetrain.enableAndResetEncoders();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
         // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 150; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        detector.enable(); // Start the detector!


        float angleWhenHanging = tracker.getAbsoluteAngle();
        waitForStart();
        //drivetrain.moveFB(24,.3,true,tracker);
        // lower robot

        /*climb.setClimberState(Climber.ClimberState.UP);
        climb.loop();
        Thread.sleep(7000);
        climb.setClimberState(Climber.ClimberState.OFF);
        climb.loop();
        drivetrain.moveLR(10, 0.1,false,tracker);*/
        drivetrain.moveFB(40, 0.5, true, tracker);


        /*
        tracker.enableAndResetEncoders();
        //TODO
        //TODO: CHECK ALL THE VALUES I WILL DIE
        //TODO

        drivetrain.moveLR(5,0.3,false,tracker);// move out of latch //
        pivotTo(angleWhenHanging + 5);//correct for difference in angle caused by dropping
        drivetrain.moveFB(18,.75,true,tracker);// drive forward until at corner of mat with samples
        //int mineralPosition = -1; // get position of gold sample (0, 1, 2) -> (left, center, right)
        */
        Thread.sleep(1000);
        if(detector.getAligned()){ // middle
            drivetrain.moveFB(19,.7,true,tracker);// knock off gold mineral
            Thread.sleep(500);
            drivetrain.moveFB(19,.7,false,tracker);// return to position
            Thread.sleep(500);
        } else{ // after turning left
            pivotTo(0);
            Thread.sleep(1000);
            if(detector.getAligned()){
                drivetrain.moveFB(26,.7,true,tracker);// knock off gold mineral
                Thread.sleep(500);
                drivetrain.moveFB(26,.7,false,tracker);// return to position //bruh idk just tweak the number until it works, and wont hit anything else i guess
            }
            else{ // after turning right
                pivotTo(90);
                Thread.sleep(1000);
                if(detector.getAligned()){
                    drivetrain.moveFB(26,.7,true,tracker);// knock off gold mineral
                    Thread.sleep(500);
                    drivetrain.moveFB(26,.7,false,tracker);// return to position
                }
            }
        }
        /*
        pivotTo(135); // Drive will then drive backwards
        drivetrain.moveFB(50,.7,false,tracker);// drive backward to waypoint (on safe auto paths map)
        drivetrain.moveFB(26,.7,true,tracker);// knock off gold mineral
        drivetrain.moveFB(26,.7,false,tracker);// return to position
        pivotTo(135); // Drive will then drive backwards
        drivetrain.moveFB(50,.7,false,tracker);// drive backward to waypoint (on safe auto paths map)
        pivotCounterclockwise(Math.abs(initialAngle)); // turn left so back is facing depot // TODO: check value, right now it is the same as the initial angle
        drivetrain.moveFB(38,.7,false,tracker);// drive backward until depot
        // TODO: deposit team marker (from back of robot)
        // (shouldn't need to) turn left to face crater
        drivetrain.moveFB(80,.7,true,tracker);// drive forward until parked on crater
        */

    }

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
                pivotClockwise(360 - (currentAngle - targetAngle));            } else if(targetAngle > currentAngle){
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