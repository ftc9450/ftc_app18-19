package org.firstinspires.ftc.teamcode.util;

/**
 * Created by dhruv on 1/22/18.
 */

public class Constants {
    private static final String vuforiakey = "AYwm7lb/////AAAAGeQI9HT4B0R2unLNBq/DsId49BJr71nKGdfP8X8fnmtD0Jna47KLigPBytLYBjzOIl6uCfYWbIXHc3FqoabxIITohKJ4VvPispe5kGGFFJyQEIifEL1Bc511jOl00pyY2Tr/YOGwjGk7lSXQ0QrScHVaiwIOM3mUUlsv9Ethn1OCZB2AVhT91gnrUKryxBwfLCGjqpmYdWOVDsJTloDiowWMez0U42S9sILVevNguQXZqTr1uURaUx5Voy+2N6FVK5p4dvraac9+LD6YskUCLqWsK2XVruCpCsRWZxfrqylNyni2ll5AW3Mekw/hSSzfjx70eyKXyaLRiOj4UhHKCjeqWjFCePt0Vb59tyqd9KhS\n";
    public class Drivetrain {
        public static final double HIGH_POWER = 1;
        public static final double LOW_POWER = 0.25;
        public static final int STRAFEINCH = 135;
        public static final int INCH = 40;//1120 cpr for neverest 40
        public static final int DEGREE = 18;
        public static final int FB_THRESHOLD = 0; // TODO: check value
        public static final int LR_THRESHOLD = 0; // TODO: check value
        public static final double FB_LEFT_POWER = 1.0; // TODO: check value
        public static final double FB_RIGHT_POWER = 1.0; // TODO: check value
        public static final double LR_FRONT_POWER = 1.0; // TODO: check value
        public static final double LR_REAR_POWER = 1.0; // TODO: check value
        public static final String LF = "frontleft";
        public static final String LB = "backleft";
        public static final String RF = "frontright";
        public static final String RB = "backright";
    }
    public class Lifter{
        public static final String LIFT = "Lifter";
        public static final String LID = "Lid";
        public static final double HIGH_POWER = 1;
        public static final double LOW_POWER = 0.25;
        public static final double PID_THRESHOLD = 10;
        public static final int UP_POSITION = 1000; // TODO: check value
        public static final double LID_CLOSED = 0.50; // TODO: check value
        public static final double LID_OPEN = 0.65; // TODO: check value
    }
    public class Climber{
        public static final String CL = "Climber";
        public static final double HIGH_POWER = 1;
        public static final double LOW_POWER = -1;
    }
    public class Intake{
        public static final String EX = "extender";
        public static final String PI = "pivot";
        public static final String RO = "roller";
        public static final double HIGH_POWER = 1;
        public static final double LOW_POWER = -1;
        public static final int IN_POSITION = 100; // TODO: check value
    }
    public class MotionTracker{
        public static final String FB = "frontleft";
        public static final String LR = "frontright";
        public static final int CLICKS_PER_INCH = 13; // cpr of 360 // TODO: check value
    }
    public class Auto{
        public static final double PIVOT_POWER = 0.3;
        public static final double PIVOT_THRESHOLD = 0.1;
    }
    public class EncoderValue{
        private static final int nevrest40 = 28;
        private static final int neverst60 =420;
        private static final double neverst20 = 134.4;
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
