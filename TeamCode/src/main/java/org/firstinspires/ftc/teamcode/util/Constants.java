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
        public static final String LF = "frontleft";
        public static final String LB = "backleft";
        public static final String RF = "frontright";
        public static final String RB = "backright";
    }
    public class Lifter{
        public static final String LI = "Lifter";
        public static final double HIGH_POWER = 1;
        public static final double LOW_POWER = 0.25;
    }
    public class Climber{
        public static final String LI = "Lifter";
        public static final double HIGH_POWER = 1;
        public static final double LOW_POWER = -1;
    }
    public class Intake{
        public static final String RO = "roller";
        public static final String BA = "base";
        public static final double HIGH_POWER = 1;
        public static final double LOW_POWER = -1;


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
