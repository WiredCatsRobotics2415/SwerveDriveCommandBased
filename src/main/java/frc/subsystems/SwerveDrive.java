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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.SwerveModuleName;
import frc.utils.RobotPreferences;

public class SwerveDrive extends SubsystemBase {
    //HARDWARE
    private AHRS navX;
    private SwerveModule[] modules;

    //PROPERTIES
    private boolean fieldOrientedEnabled = true;

    //STATES
    private SwerveDriveOdometry odometry;

    public SwerveDrive() {
        modules = new SwerveModule[] {
            new SwerveModule(SwerveModuleName.FRONT_LEFT),
            new SwerveModule(SwerveModuleName.FRONT_RIGHT),
            new SwerveModule(SwerveModuleName.BACK_LEFT),
            new SwerveModule(SwerveModuleName.BACK_RIGHT)
        };

        navX = new AHRS(Port.kMXP);
        navX.reset();

        odometry = new SwerveDriveOdometry(
            Constants.Swerve.KINEMATICS,
            navX.getRotation2d(),
            new SwerveModulePosition[] {
                modules[0].getPosition(),
                modules[1].getPosition(),
                modules[2].getPosition(),
                modules[3].getPosition(),
            }
        );
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
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, navX.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot));
        
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_DRIVE_SPEED);
        for (int i = 0; i < 4; i++) {
            modules[i].setNewState(swerveModuleStates[i]);
        }
    }

    public void setFieldOriented(boolean enabled) {
        fieldOrientedEnabled = enabled;
    }

    public void resetHeading() {
        navX.reset();
    }

    @Override
    public void periodic() {
        odometry.update(Rotation2d.fromDegrees(navX.getAngle()+RobotPreferences.getNavXOffset()), new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition(),
        });

        for (SwerveModule m : modules) {
            SmartDashboard.putData(m.name.name(), m);
        }
    }

    public SwerveModule[] getModules() {
        return modules;
    }
}
