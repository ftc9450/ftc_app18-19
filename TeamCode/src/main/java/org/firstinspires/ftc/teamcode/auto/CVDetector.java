package org.firstinspires.ftc.teamcode.auto;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SilverDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.opencv.core.Mat;
import org.opencv.core.Size;

public class CVDetector extends OpMode {
    private GoldDetector gd;
    private SilverDetector sd;

    @Override
    public void init() {
        gd = new GoldDetector();
        sd = new SilverDetector();
        gd.setAdjustedSize(new Size(480, 720));
        gd.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        gd.useDefaults();
        gd.downscale = 0.4;
        gd.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
        gd.maxAreaScorer.weight = 0.005;
        gd.ratioScorer.weight = 5;
        gd.ratioScorer.perfectRatio = 1.0;
        gd.enable();

        sd.setAdjustedSize(new Size(480, 720));
        sd.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        sd.useDefaults();
        sd.downscale = 0.4;
        sd.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
        sd.maxAreaScorer.weight = 0.005;
        sd.ratioScorer.weight = 5;
        sd.ratioScorer.perfectRatio = 1.0;
        sd.enable();
    }

    @Override
    public void loop() {
        double p[] = run(new Mat());
        telemetry.addData("gold pos:", p[0] + ", " + p[1]);
    }

    public double[] run(Mat img) {
        gd.process(img);
        sd.process(img);
        return new double[]{gd.getScreenPosition().x, gd.getScreenPosition().y};
    }

    public void stop() {
        gd.disable();
        sd.disable();
    }
}
