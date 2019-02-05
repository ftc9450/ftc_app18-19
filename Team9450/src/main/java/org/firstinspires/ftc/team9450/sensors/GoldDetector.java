package org.firstinspires.ftc.team9450.sensors;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.team9450.util.Constants;

import java.util.List;

public class GoldDetector {
    VuforiaLocalizer vuforia;
    TFObjectDetector tfod;
    Recognition gold;
    List<Recognition> recs;

    public GoldDetector(int tfodMonitorViewId) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = Constants.Vuforia.KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset("RoverRuckus.tflite", "Gold Mineral", "Silver Mineral");
    }

    /**
     * process frames
     * @return gold detected
     */
    public int process() {
        tfod.activate();
        recs = tfod.getUpdatedRecognitions();
        if (recs != null) {
            if (recs.size() > 0) gold = recs.get(0);
            return recs.size();
        }
        return -1;
        /*if (recs != null && recs.size() > 0) {
            gold = recs.get(0);
            return true;
            /*for (Recognition rec:recs) {
                if (rec.getLabel().equals("Gold Mineral")) {
                    gold = rec;
                    return true;
                }
            }*/
        //}
    } //TODO: always equal to false

    public double getAngle() {
        return gold.estimateAngleToObject(AngleUnit.DEGREES);
    }

    public double getDistance() {
        return gold.getHeight();
    }
}
