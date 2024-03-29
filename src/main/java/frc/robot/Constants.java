package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.utils.PIDValue;

public class Constants {
    /**
    * The amount of time to wait for a CAN ack message. Increase if there are too many CAN errors. if there are still to many and CAN utilization is low, have the wiring inspected
    **/
    public static final int CAN_TIMEOUT_MS = 30;

    //Package in its own namespace so this code can easily be copied and pasted into other robot code
    public static class Swerve {
        public enum SwerveModuleName {
            FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT;
        }

        //Auto azimuth sync settings
        public static final int ENCODER_RESET_TICKS = 30;
        public static final int ENCODER_RESET_DEGREE_THRESHOLD = 1;
        public static double ENCODER_RESET_CPR_THRESHOLD = 0;
        static {
            ENCODER_RESET_CPR_THRESHOLD = degreesToFalcon(ENCODER_RESET_DEGREE_THRESHOLD)*.1;
        }

        //Physical module and drivebase dimentions
        public static final double AZIMUTH_GEAR_RATIO = (26.0 / 9.0) * (96.0 / 18.0); 
        public static final double DRIVE_GEAR_RATIO = (26.0 / 9.0) * (45 / 15.0); 
        public static final double WHEEL_CIRCUMFERENCE = Math.PI * Units.inchesToMeters(5);
        public static final double DRIVE_DISTANCE_PER_ROTATION = WHEEL_CIRCUMFERENCE/DRIVE_GEAR_RATIO; 

        public static final double LEFTRIGHT_DISTANCE = Units.inchesToMeters(29.9);
        public static final double FRONTBACK_DISTANCE = Units.inchesToMeters(29.9);
        public static final double WHEEL_DIST = Math.sqrt((LEFTRIGHT_DISTANCE * LEFTRIGHT_DISTANCE) + (FRONTBACK_DISTANCE * FRONTBACK_DISTANCE));

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(FRONTBACK_DISTANCE / 2.0, LEFTRIGHT_DISTANCE / 2.0),
            new Translation2d(FRONTBACK_DISTANCE / 2.0, -LEFTRIGHT_DISTANCE / 2.0),
            new Translation2d(-FRONTBACK_DISTANCE / 2.0, LEFTRIGHT_DISTANCE / 2.0),
            new Translation2d(-FRONTBACK_DISTANCE / 2.0, -LEFTRIGHT_DISTANCE / 2.0)
        );

        //Drive motor control gains (currently not used)
        public static final SimpleMotorFeedforward DRIVE_FF = new SimpleMotorFeedforward(0.271, 0.928, 0.08);
        public static final PIDValue DRIVE_PID = new PIDValue(0.0767,0,0);

        //Current limits
        public static final SupplyCurrentLimitConfiguration DRIVE_CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(false, 30, 60, 0.1);
        public static final SupplyCurrentLimitConfiguration AZIMUTH_CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(true, 25, 40, 0.1);

        public static final PIDValue[] AZIMUTH_PIDS = {
            new PIDValue(0.14, 0.0, .012), //Front Left
            new PIDValue(0.14, 0.0, .012), //Front Right
            new PIDValue(0.14, 0.0, .012), //Back Left
            new PIDValue(0.16, 0.0, .012), //Back Right
        };

        //Maximum swerve speed
        public static final double MAX_ANGULAR_SPEED = 3.0; //m/s
        public static final double MAX_DRIVE_SPEED = 8.0; //m/s

        /**
         * Converting a degree value to a falcon integrated encoder 2048 tick value. Note that 
         * this does <b>not</b> compensate for a motor's encoder value reading more than 2048 and 
         * should be used in conjunction with a function such as over360RangeSetter
         */
        public static double degreesToFalcon(double degrees) {
            return degrees / (360.0 / (AZIMUTH_GEAR_RATIO * 2048.0));
        }
    
        public static double falconToDegrees(double ticks) {
            return ticks * (360.0 / (AZIMUTH_GEAR_RATIO * 2048.0));
        }
    
        public static double driveFalconToDegrees(double ticks) {
            return ticks * (WHEEL_CIRCUMFERENCE / (DRIVE_GEAR_RATIO * 2048.0));
        }
        
        public static double driveFalconToDistance(double ticks) {
            return (ticks/2048) * DRIVE_DISTANCE_PER_ROTATION;
        }

        public static double falconToRPM(double velocityCounts) {
            return (velocityCounts * (600.0 / 2048.0)) / DRIVE_GEAR_RATIO;
        }
    
        public static double RPMToFalcon(double RPM) {
            return (RPM * DRIVE_GEAR_RATIO) * (2048.0 / 600.0);
        }
    
        public static double falconToMPS(double velocitycounts) {
            return (falconToRPM(velocitycounts) * WHEEL_CIRCUMFERENCE) / 60.0;
        }
    
        public static double MPSToFalcon(double velocity) {
            return RPMToFalcon((velocity * 60) / WHEEL_CIRCUMFERENCE);
        }
    }
}
