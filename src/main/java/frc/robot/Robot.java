// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private RobotContainer robot;
  private EventLoop eventLoop;
  private Command auto;

  @Override
  public void robotInit() {
    eventLoop = new EventLoop();
    robot = new RobotContainer(eventLoop);
  }

  @Override
  public void teleopInit() {
    robot.teleopInit();
  }

  @Override
  public void autonomousInit() {
    auto = robot.getAuto();
    if (auto != null) CommandScheduler.getInstance().schedule(auto);
  }

  @Override
  public void autonomousExit() {
    if (auto != null) auto.cancel();
  }

  @Override
  public void robotPeriodic() {
    eventLoop.poll();
    CommandScheduler.getInstance().run();
  }
}

//Needs testing
//"Theory will only take you so far" - Oppenheimer