// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LightingSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final LightingSubsystem lightingSubsystem = LightingSubsystem.getInstance();
  private final EndEffectorSubsystem endEffectorSubsystem = EndEffectorSubsystem.getInstance();
  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
    lightingSubsystem.FlashAllianceColour();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    lightingSubsystem.FlashAllianceColour();
    lightingSubsystem.RainbowAnimateSide();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    lightingSubsystem.StrobeAnimate(0, 255, 0);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    lightingSubsystem.AnimateTeam();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    if (endEffectorSubsystem.getSensor()) {
      lightingSubsystem.StrobeAnimate(0, 255, 0);
      lightingSubsystem.FlashingWhite();
    }
    else {
      lightingSubsystem.setLEDs(255, 0, 0);
    }
  }

  @Override
  public void teleopExit() {
    lightingSubsystem.FlashAllianceColour();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
