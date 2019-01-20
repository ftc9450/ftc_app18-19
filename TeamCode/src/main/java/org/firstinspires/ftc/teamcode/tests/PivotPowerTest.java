package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.sensors.Gyroscope;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.MotionTracker;

@TeleOp
public class PivotPowerTest extends OpMode {

    private Drivetrain drivetrain;
    private Gyroscope imu;
    private MotionTracker tracker;
    private double pivotPower = 0.1;
    private boolean aPressed;
    private boolean bPressed;
    private boolean xPressed;
    private boolean yPressed;
    private double MAX_POWER = Constants.Auto.MAX_PIVOT_POWER;
    private double MIN_POWER = Constants.Auto.MIN_PIVOT_POWER;
    private Mode mode;
    public enum Mode{
        MAX,MIN
    }

    public void init(){
        imu = new Gyroscope(hardwareMap.get(BNO055IMU.class, "imu"));
        drivetrain = new Drivetrain(hardwareMap.dcMotor.get(Constants.Drivetrain.LF),hardwareMap.dcMotor.get(Constants.Drivetrain.LB), hardwareMap.dcMotor.get(Constants.Drivetrain.RF), hardwareMap.dcMotor.get(Constants.Drivetrain.RB));
        tracker = new MotionTracker(hardwareMap.dcMotor.get(Constants.MotionTracker.FB), hardwareMap.dcMotor.get(Constants.MotionTracker.LR), drivetrain, imu, 0);

    }
    public void loop(){

        if(gamepad1.dpad_up){
            mode = Mode.MAX;
        } else if(gamepad1.dpad_down){
            mode = Mode.MIN;
        }

        if(gamepad1.x){
            if(!xPressed){
                switch(mode){
                    case MAX:
                        MAX_POWER += 0.01;
                        break;
                    case MIN:
                        MIN_POWER += 0.01;
                }
            }
            xPressed = true;
        } else{
            xPressed = false;
        }
        if(gamepad1.y){
            if(!yPressed) {
                switch(mode){
                    case MAX:
                        MAX_POWER -= 0.01;
                        break;
                    case MIN:
                        MIN_POWER -= 0.01;
                }
                yPressed = true;
            }
        } else{
            yPressed = false;
        }
        if(gamepad1.right_bumper){
            drivetrain.setState(Drivetrain.DrivetrainState.Turning);
            float startAngle = tracker.getAbsoluteAngle();
            double power = mode == Mode.MAX? MAX_POWER: MIN_POWER;
            double[] driveSignal = new double[]{power,power,-power,-power};
            drivetrain.setPower(driveSignal);
        }else if(gamepad1.left_bumper){
            drivetrain.setState(Drivetrain.DrivetrainState.Turning);
            float startAngle = tracker.getAbsoluteAngle();
            double power = mode == Mode.MAX? MAX_POWER: MIN_POWER;
            double[] driveSignal = new double[]{-power,-power,power,power};
            drivetrain.setPower(driveSignal);
        } else{
            drivetrain.setPower(0);
        }

        telemetry.addData("x: ",tracker.getXEncoderValue());
        telemetry.addData("y: ", tracker.getYEncoderValue());
        telemetry.addData("angle: ", imu.getAngle());
        telemetry.update();



    }
}
