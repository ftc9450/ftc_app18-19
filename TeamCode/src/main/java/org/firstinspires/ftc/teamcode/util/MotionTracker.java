package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.sensors.Gyroscope;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Constants;

/**
 * Created by Avi Trost on 12/15/18.
 */

public class MotionTracker {
    public int x;
    public int y;
    private float angle;
    private int previousX;
    private int previousY;
    private int xWhileTurning;
    private int yWhileTurning;
    private float previousAngle;
    private DcMotor xOmni;
    private DcMotor yOmni;
//  private Gyroscope gyro;
    private Drivetrain drivetrain;
    public MotionTracker(DcMotor forwardOmniWheel, DcMotor sidewaysOmniWheel, Drivetrain drivetrain){
        this.drivetrain = drivetrain;

//      angle = gyro.getAngle();
//      previousAngle = angle;

        xOmni = forwardOmniWheel;
        yOmni = sidewaysOmniWheel;

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
        previousX = x;
        previousY = y;
        x = xOmni.getCurrentPosition() - xWhileTurning;
        y = yOmni.getCurrentPosition() - yWhileTurning;
//      previousAngle = angle;
//      angle = gyro.getAngle();
        if(drivetrain.getState() == Drivetrain.DrivetrainState.Turning){ // Ensures encoder values while rotating
            xWhileTurning += x - previousX;                              // will have no false impact on position
            yWhileTurning += y - previousY;
            x = previousX;
            y = previousY;
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
