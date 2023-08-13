package frc.robot;

public class RobotMap {
    public static class Swerve {
        public static final int FRONT_LEFT_DRIVE = 4;
        public static final int FRONT_RIGHT_DRIVE = 1;
        public static final int BACK_LEFT_DRIVE = 6;
        public static final int BACK_RIGHT_DRIVE = 8;
        public static final int[] DRIVE_PORTS = { FRONT_LEFT_DRIVE, FRONT_RIGHT_DRIVE, BACK_LEFT_DRIVE, BACK_RIGHT_DRIVE };

        public static final boolean FRONT_LEFT_DRIVE_REV = false;
        public static final boolean FRONT_RIGHT_DRIVE_REV = true;
        public static final boolean BACK_LEFT_DRIVE_REV = false;
        public static final boolean BACK_RIGHT_DRIVE_REV = true;
        public static final boolean[] DRIVE_REVERSED = { FRONT_LEFT_DRIVE_REV, FRONT_RIGHT_DRIVE_REV, BACK_LEFT_DRIVE_REV, BACK_RIGHT_DRIVE_REV };

        public static final int FRONT_LEFT_AZIMUTH = 3;
        public static final int FRONT_RIGHT_AZIMUTH = 7;
        public static final int BACK_LEFT_AZIMUTH = 2;
        public static final int BACK_RIGHT_AZIMUTH = 5;
        public static final int[] AZIMUTH_PORTS = { FRONT_LEFT_AZIMUTH, FRONT_RIGHT_AZIMUTH, BACK_LEFT_AZIMUTH, BACK_RIGHT_AZIMUTH };

        public static final boolean FRONT_LEFT_AZIMUTH_REV = false;
        public static final boolean FRONT_RIGHT_AZIMUTH_REV = false;
        public static final boolean BACK_LEFT_AZIMUTH_REV = false;
        public static final boolean BACK_RIGHT_AZIMUTH_REV = false;
        public static final boolean[] AZIMUTH_REVERSED = { FRONT_LEFT_AZIMUTH_REV, FRONT_RIGHT_AZIMUTH_REV, BACK_LEFT_AZIMUTH_REV, BACK_RIGHT_AZIMUTH_REV };

        public static final int FRONT_LEFT_ABSOLUTE_ENCODER = 3;
        public static final int FRONT_RIGHT_ABSOLUTE_ENCODER = 1;
        public static final int BACK_LEFT_ABSOLUTE_ENCODER = 2;
        public static final int BACK_RIGHT_ABSOLUTE_ENCODER = 0;
        public static final int[] ABSOLUTE_ENCODER_PORTS = { FRONT_LEFT_ABSOLUTE_ENCODER, FRONT_RIGHT_ABSOLUTE_ENCODER,
                BACK_LEFT_ABSOLUTE_ENCODER, BACK_RIGHT_ABSOLUTE_ENCODER };

        public static final boolean FRONT_LEFT_ABSOLUTE_ENCODER_REV = true;
        public static final boolean FRONT_RIGHT_ABSOLUTE_ENCODER_REV = true;
        public static final boolean BACK_LEFT_ABSOLUTE_ENCODER_REV = true;
        public static final boolean BACK_RIGHT_ABSOLUTE_ENCODER_REV = true;
        public static final boolean[] ABSOLUTE_ENCODER_REV = { FRONT_LEFT_ABSOLUTE_ENCODER_REV, FRONT_RIGHT_ABSOLUTE_ENCODER_REV, BACK_LEFT_ABSOLUTE_ENCODER_REV, BACK_RIGHT_ABSOLUTE_ENCODER_REV };
    }
}
