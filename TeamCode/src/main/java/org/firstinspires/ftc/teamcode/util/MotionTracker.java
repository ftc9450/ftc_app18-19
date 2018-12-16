package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MotionTracker {
    public int x;
    public int y;
    private DcMotor xOmni;
    private DcMotor yOmni;
    public MotionTracker(DcMotor forwardOmniWheel, DcMotor sidewaysOmniWheel){
        x = 0;
        y = 0;

        xOmni = forwardOmniWheel;
        yOmni = sidewaysOmniWheel;

        xOmni.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        xOmni.setDirection(DcMotor.Direction.FORWARD);
        yOmni.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        yOmni.setDirection(DcMotor.Direction.FORWARD);
    }

    public void updatePosition(){
        x = xOmni.getCurrentPosition();
        y = yOmni.getCurrentPosition();
    }
}
