// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.subsystems.SwerveDrive;
import frc.subsystems.SwerveModule;
import frc.utils.RobotPreferences;

public class Robot extends TimedRobot {
  private final SwerveDrive swerveDrive = new SwerveDrive();

  private SendableChooser<Integer> oiChooser;
  private OIs.OI selectedOI;
  private boolean isPressingUserButton = false;

  @Override
  public void robotInit() {
    oiChooser = new SendableChooser<Integer>();
    oiChooser.setDefaultOption("Gulikit Controller", 0);
    for (String k : Preferences.getKeys()) {
      System.out.println(k);
    }
  }

  private void configButtonBindings() {
    selectedOI.binds.get("navX Reset").onTrue(new InstantCommand(() -> {
      swerveDrive.resetHeading();
    }, swerveDrive));
  }

  @Override
  public void teleopInit() {
    switch (oiChooser.getSelected()) {
      case 0:
        selectedOI = new OIs.GulikitController();
        break;
      default:
        selectedOI = new OIs.GulikitController();
        break;
    }
    selectedOI.setInputPrefrences();
    configButtonBindings();
    swerveDrive.setFieldOriented(RobotPreferences.getFieldOriented());
    swerveDrive.setDefaultCommand(new RunCommand(() -> {
      swerveDrive.drive(selectedOI.getX(), selectedOI.getY(), selectedOI.getRotation());
    }, swerveDrive));
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putData("OI", oiChooser);
    CommandScheduler.getInstance().run();
    if (RobotController.getUserButton()) {
      if (!isPressingUserButton) {
        isPressingUserButton = true;
        System.out.println("Zeroing...");
        //Zero time
        for (SwerveModule m : swerveDrive.getModules()) {
          RobotPreferences.setOffsetOfModule(m.name, m.getState().angle.getDegrees());
          m.setModuleOffsetFromStorage();
        }
        System.out.println("Done Zeroing");
      }
    } else {
      if (isPressingUserButton) isPressingUserButton = false;
    }
  }
}
