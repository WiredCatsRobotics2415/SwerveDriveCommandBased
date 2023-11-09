// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
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

  //#region COMMANDS
  InstantCommand zeroCommand = new InstantCommand(new Runnable() {
    @Override
    public void run() {
      System.out.println("Zeroing...");
      for (SwerveModule m : swerveDrive.getModules()) {
        RobotPreferences.setOffsetOfModule(m.name, m.getState().angle.getDegrees());
        m.setModuleOffsetFromStorage();
      }
      System.out.println("Done zeroing...");
    }
  }, swerveDrive);
  //#endregion

  @Override
  public void robotInit() {
    oiChooser = new SendableChooser<Integer>();
    oiChooser.setDefaultOption("Gulikit Controller", 0);
    for (String k : Preferences.getKeys()) System.out.println(k);
  }

  private void configButtonBindings() {
    selectedOI.binds.get("navX Reset").onTrue(new InstantCommand(() -> {
      swerveDrive.resetHeading();
    }, swerveDrive));
  }

  @Override
  public void teleopInit() {
    // if (DriverStation.isFMSAttached()) {
    //  //If in a comp setting, this method runs right after auto is over and teleop is enabled
    //  //The perfect time to offset the navX so the driver doesn't waste time doing anything
    //   RobotPreferences.setNavXOffset(180);
    // }
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
    SmartDashboard.putData("Zero", zeroCommand);
    if (RobotController.getUserButton()) {
      if (!isPressingUserButton) {
        isPressingUserButton = true;
        CommandScheduler.getInstance().schedule(zeroCommand);
      }
    } else {
      if (isPressingUserButton) isPressingUserButton = false;
    }
    CommandScheduler.getInstance().run();
  }
}

//Needs testing
//"Theory will only take you so far" - Oppenheimer