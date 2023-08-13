package frc.utils;

import edu.wpi.first.wpilibj.Preferences;
import frc.robot.Constants;

public class RobotPreferences {
    public static enum PrefTypes {
        BOOL,
        DOUBLE,
        FLOAT,
        LONG,
        INT,
        STRING
    }

    private static void ensureExistance(String key, PrefTypes type, Object defaultValue) {
        if (!Preferences.containsKey(key)) {
            switch (type) {
                case BOOL:
                    Preferences.initBoolean(key, (boolean)defaultValue);
                    break;
                case DOUBLE:
                    Preferences.initDouble(key, (double)defaultValue);
                    break;
                case FLOAT:
                    Preferences.initFloat(key, (float)defaultValue);
                    break;
                case INT:
                    Preferences.initInt(key, (int)defaultValue);
                    break;
                case LONG:
                    Preferences.initLong(key, (long)defaultValue);
                    break;
                case STRING:
                    Preferences.initString(key, (String)defaultValue);
                    break;
                default:
                    break;
            }
        }
    }

    /**
    * @return true if preference is set to curve, false if preference is slew
    */
    public static boolean getInputFilter() {
        String key = "Input Filter Type (True for curve, False for slew): ";
        ensureExistance(key, PrefTypes.BOOL, true);
        return Preferences.getBoolean(key, true);
    }

    /**
    * @return integer>0 if IF is true, otherwise 1
    */
    public static int getCurvePower() {
        String key = "Curve Power (Only if IF is True): ";
        ensureExistance(key, PrefTypes.INT, 1);
        return Preferences.getInt(key, 1);
    }

    /**
    * @return double>0 if IF is false, otherwise 1
    */
    public static double getSlewRateLimit() {
        String key = "Slew limit (Only if IF is False): ";
        ensureExistance(key, PrefTypes.DOUBLE, 1);
        return Preferences.getDouble(key, 1.0d);
    }

    /**
    * @return the angle offset of the swerve module passed in
    */
    public static double getOffsetOfModule(Constants.Swerve.SwerveModuleName name) {
        switch (name) {
            case FRONT_LEFT:
                String keyFL = "FL_OFFSET (DO NOT TOUCH)";
                ensureExistance(keyFL, PrefTypes.DOUBLE, 0.0);
                return Preferences.getDouble(keyFL, 0.0);
            case BACK_LEFT:
                String keyBL = "BL_OFFSET (DO NOT TOUCH)";
                ensureExistance(keyBL, PrefTypes.DOUBLE, 0.0);
                return Preferences.getDouble(keyBL, 0.0);
            case BACK_RIGHT:
                String keyBR = "BR_OFFSET (DO NOT TOUCH)";
                ensureExistance(keyBR, PrefTypes.DOUBLE, 0.0);
                return Preferences.getDouble(keyBR, 0.0);
            case FRONT_RIGHT:
                String keyFR = "FR_OFFSET (DO NOT TOUCH)";
                ensureExistance(keyFR, PrefTypes.DOUBLE, 0.0);
                return Preferences.getDouble(keyFR, 0.0);
            default:
                return 0.0;
        }
    }

    /**
    * @param name name of the module to offset
    * @param offset offset as a double
    */
    public static void setOffsetOfModule(Constants.Swerve.SwerveModuleName name, double offset) {
        switch (name) {
            case FRONT_LEFT:
                String keyFL = "FL_OFFSET (DO NOT TOUCH)";
                ensureExistance(keyFL, PrefTypes.DOUBLE, 0.0);
                Preferences.setDouble(keyFL, offset);
            case BACK_LEFT:
                String keyBL = "BL_OFFSET (DO NOT TOUCH)";
                ensureExistance(keyBL, PrefTypes.DOUBLE, 0.0);
                Preferences.setDouble(keyBL, offset);
            case BACK_RIGHT:
                String keyBR = "BR_OFFSET (DO NOT TOUCH)";
                ensureExistance(keyBR, PrefTypes.DOUBLE, 0.0);
                Preferences.setDouble(keyBR, offset);
            case FRONT_RIGHT:
                String keyFR = "FR_OFFSET (DO NOT TOUCH)";
                ensureExistance(keyFR, PrefTypes.DOUBLE, 0.0);
                Preferences.setDouble(keyFR, offset);
            default:
                break;
        }
    }

    /**
     * Should the robot drive relative to itself or to the field?
     * @return True if field oriented, false if robot oriented
     */
    public static boolean getFieldOriented() {
        String key = "Field Oriented Drive: ";
        ensureExistance(key, PrefTypes.BOOL, false);
        return Preferences.getBoolean(key, true);
    }

    /**
     * What angle should the navX be offset by?
     * ie. set to 180 if the robot starts backwards
     * @return int of offset in degrees, default 0
     */
    public static int getNavXOffset() {
        String key = "navX angle offset";
        ensureExistance(key, PrefTypes.INT, 0);
        return Preferences.getInt(key, 0);
    }

     /**
      * What angle should the navX be offset by?
      * ie. set to 180 if the robot starts backwards
      * @param offset offset in degrees
     */
    public static void setNavXOffset(int offset) {
        String key = "navX angle offset";
        ensureExistance(key, PrefTypes.INT, offset);
        Preferences.setInt(key, offset);
    }
}
