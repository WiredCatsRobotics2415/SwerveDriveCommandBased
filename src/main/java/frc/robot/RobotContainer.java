package frc.robot;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OIs.OI;
import frc.subsystems.SwerveDrive;
import frc.utils.Logger;
import frc.utils.RobotPreferences;
import frc.utils.Logger.LogLevel;

public class RobotContainer {
    private final SwerveDrive swerveDrive = new SwerveDrive();

    private OIs.OI selectedOI;
    private boolean isPressingUserButton = false;
    private EventLoop eventLoop;

    public RobotContainer(EventLoop loop) {
        eventLoop = loop;
        for (String k : Preferences.getKeys()) Logger.log(LogLevel.INFO, k);
        SmartDashboard.putData("Zero", new InstantCommand(() -> swerveDrive.zeroAllModules()).ignoringDisable(true));
        BooleanEvent userButtonPressing = new BooleanEvent(eventLoop, RobotController::getUserButton);
        userButtonPressing.falling().ifHigh(() -> {
            swerveDrive.zeroAllModules();
        });
    }

    private void configButtonBindings() {
        selectedOI.binds.get("navX Reset").onTrue(new InstantCommand(() -> {
            swerveDrive.resetHeading();
        }, swerveDrive));
    }

    //Calls methods from subsystems to update from preferences
    private void configurePreferences() {
        selectedOI.setInputPrefrences();
        swerveDrive.setFieldOriented(RobotPreferences.getFieldOriented());
    }

    public void teleopInit() {
        //Gets the OI selected
        switch (OI.oiChooser.getSelected()) {
            case 0:
              selectedOI = new OIs.GulikitController();
              break;
            default:
              selectedOI = new OIs.GulikitController();
              break;
        }
        configurePreferences();
        configButtonBindings();
        swerveDrive.setDefaultCommand(new RunCommand(() -> {
            swerveDrive.drive(selectedOI.getX(), selectedOI.getY(), selectedOI.getRotation());
        }, swerveDrive));
    }

    public Command getAuto() {
        return null;
    }
}
