// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.SwerveModuleName;
import frc.utils.Logger;
import frc.utils.RobotPreferences;
import frc.utils.Logger.LogLevel;

public class SwerveDrive extends SubsystemBase {
    //HARDWARE
    private AHRS navX;
    private SwerveModule[] modules;

    //PROPERTIES
    private boolean fieldOrientedEnabled = true;

    //STATES
    private SwerveDriveOdometry odometry;
    private Field2d field;
    private boolean isPreparedForZero = false;

    public SwerveDrive() {
        modules = new SwerveModule[] {
            new SwerveModule(SwerveModuleName.FRONT_LEFT),
            new SwerveModule(SwerveModuleName.FRONT_RIGHT),
            new SwerveModule(SwerveModuleName.BACK_LEFT),
            new SwerveModule(SwerveModuleName.BACK_RIGHT)
        };

        navX = new AHRS(Port.kMXP);
        resetHeading();

        odometry = new SwerveDriveOdometry(
            Constants.Swerve.KINEMATICS,
            getYaw(),
            new SwerveModulePosition[] {
                modules[0].getPosition(),
                modules[1].getPosition(),
                modules[2].getPosition(),
                modules[3].getPosition(),
            }
        );
        field = new Field2d();
    }

    public void zeroAllModules() {
        for (SwerveModule m : getModules()) {
            if (!isPreparedForZero) {
                m.setModuleOffset(0);
            } else {
                double offset = m.getState().angle.getDegrees();
                RobotPreferences.setOffsetOfModule(m.name, offset);
                m.setModuleOffset(offset);
            }
        }
        if (!isPreparedForZero) {
            Logger.log(LogLevel.WARNING, "Zero preparation entered");
            isPreparedForZero = true;
        } else {
            Logger.log(LogLevel.WARNING, "Done zeroing");
            isPreparedForZero = false;
        }
    }

    /**
    * Sets the states of all of the modules. Expects the inputs to be between -1 and 1, ie. from the joystick input filters
    * @param xSpeed X axis speed
    * @param ySpeed Y axis speed
    * @param rot Rotational speed
    */
    public void drive(double xSpeed, double ySpeed, double rot) {
        xSpeed *= Constants.Swerve.MAX_DRIVE_SPEED;
        ySpeed *= Constants.Swerve.MAX_DRIVE_SPEED;
        rot *= Constants.Swerve.MAX_ANGULAR_SPEED;

        SwerveModuleState[] swerveModuleStates = Constants.Swerve.KINEMATICS.toSwerveModuleStates(
                this.fieldOrientedEnabled
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getYaw())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot));
        
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_DRIVE_SPEED);
        for (int i = 0; i < 4; i++) {
            modules[i].setNewState(swerveModuleStates[i]);
        }
    }

    public void setFieldOriented(boolean enabled) {
        fieldOrientedEnabled = enabled;
    }

    public Rotation2d getYaw() {
        double yaw = navX.getYaw();
        yaw *= -1;
        // The navX returns an angle between (-180, 180) where 0 is forward, and
        // positive is counterclockwise.
        // The WPILib swerve relies on an angle between [0, 360) where 0 is forwards and
        // positive is counterclockwise.
        // This function changes the navX return angle to fit the WPILib swerve angle.
        if (yaw < 0) {
            yaw = 360 - (yaw * -1);
        }
        //always positive yaw
        yaw = (yaw % 360 + 360) % 360;
        
        return Rotation2d.fromDegrees(yaw+90);
    }

    public void resetHeading() {
        navX.zeroYaw();
    }

    @Override
    public void periodic() {
        odometry.update(getYaw(), new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition(),
        });
        field.setRobotPose(odometry.getPoseMeters());
        SmartDashboard.putData(field);
        for (SwerveModule m : modules) {
            SmartDashboard.putData(m.name.name(), m);
        }
    }

    public SwerveModule[] getModules() {
        return modules;
    }
}
