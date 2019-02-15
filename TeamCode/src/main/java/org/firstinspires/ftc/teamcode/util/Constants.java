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
        public static final int FB_THRESHOLD = 500; // TODO: check value
        public static final int LR_THRESHOLD = 500; // TODO: check value
        public static final int FB_PID_THRESHOLD = 500; // TODO: check value
        public static final int LR_PID_THRESHOLD = 500; // TODO: check value
        public static final double FB_LEFT_POWER = 1.0; // TODO: check value
        public static final double FB_RIGHT_POWER = 1.0; // TODO: check value
        public static final double LR_FRONT_POWER = 1.0; // TODO: check value
        public static final double LR_REAR_POWER = 0.9; // TODO: check value
        public static final String LF = "frontleft";
        public static final String LB = "backleft";
        public static final String RF = "frontright";
        public static final String RB = "backright";
    }
    @Deprecated
    public class Lifter{
        public static final String EXT = "extender";
        public static final String DUMP = "dumper";
        public static final double HIGH_POWER = 1;
        public static final double LOW_POWER = 0.25;
        public static final double PID_THRESHOLD = 10;
        public static final int UP_POSITION = 1000; // TODO: check value
        public static final double DUMP_DOWN = 0.50; // TODO: check value
        public static final double DUMP_UP = 0.65; // TODO: check value
    }
    public class Climber{
        public static final String EL = "climber";
        public static final String PI = "deposit";
        public static final String HK = "hook";
        public static final String PL="pawl";
        public static final double PAWL_IN=0.26;
        public static final double PAWL_OUT=0.45;
        public static final double HOOK_OPEN=0.25;
        public static final double HOOK_CLOSED=0.74;
        public static final int UP = 21000;
        public static final int CLIMBED=10838;
        public static final int DOWN = 0;
    }
    public class Intake{
        public static final String PI = "pivot";
        public static final String RO = "roller";
        public static final double HIGH_POWER = 1;
        public static final double LOW_POWER = -1;
        public static final double PIVOT_UP = .5; // TODO: check value
        public static final double PIVOT_DOWN = .5; // TODO: check value
        public static final double PIVOT_OFF = .5; // TODO: check value
        public static final double ROllER_IN = .5; // TODO: check value
        public static final double ROLLER_OFF = .5; // TODO: check value
        public static final double ROLLER_OUT = .5; // TODO: check value
        public static final int SLIDE_IN = 200; // TODO: check value
        public static final int SLIDE_OFF = 1000; // TODO: check value
        public static final int SLIDE_OUT = 1500; // TODO: check value



    }
    public class MotionTracker{
        public static final String FB = Drivetrain.RB;
        public static final String LR = "frontleft";
        public static final double CLICKS_PER_INCH = 114.59/5; // cpr of 1440
    }
    public class Auto{
        public static final double MAX_PIVOT_POWER = 0.3;
        public static final double MIN_PIVOT_POWER = 0.1;
        public static final double PIVOT_POWER=0.2;
        public static final double PIVOT_THRESHOLD = 0.1;
        public static final String MA = "marker";
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
