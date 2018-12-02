package org.firstinspires.ftc.teamcode.util;

/**
 * Created by dhruv on 1/22/18.
 */

public class Constants {
    public class Drivetrain {
        public static final double HIGH_POWER = 1;
        public static final double LOW_POWER = 0.25;
        public static final int STRAFEINCH = 135;
        public static final int INCH = 40;//1120 cpr for neverest 40
        public static final int DEGREE = 18;
        public static final String LF = "frontleft";
        public static final String LB = "backleft";
        public static final String RF = "frontright";
        public static final String RB = "backright";
    }
    public static double floatToDouble(float f) {
        Float d=new Float(f);
        return d.doubleValue();
    }

    public static float doubleToFloat(double d){
        Double f=new Double(d);
        return f.floatValue();
    }
}
