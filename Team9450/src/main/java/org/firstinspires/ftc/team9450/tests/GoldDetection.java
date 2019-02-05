package org.firstinspires.ftc.team9450.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team9450.sensors.GoldDetector;

@TeleOp
public class GoldDetection extends OpMode {
    GoldDetector gold;
    int n;
    int s;

    @Override
    public void init() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        gold = new GoldDetector(tfodMonitorViewId);
        n = -1;
    }

    @Override
    public void loop() {
        /*if (gold.process()) {
            telemetry.addData("angle", gold.getAngle());
            telemetry.addData("distance", gold.getDistance());
        } else {
            telemetry.addLine("gold not detected.");
        }*/
        s = gold.process();
        //n = s != -1? s:n;
        if (s != -1) n = s;
        telemetry.addData("#", n);
        if (n > 0) telemetry.addData("angle", gold.getAngle());
    }
}
