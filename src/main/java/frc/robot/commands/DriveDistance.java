// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Drivetrain;
import frc.robot.subsystems.buttonCommandTest;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveDistance extends Command {
  private buttonCommandTest l_ButtonCommandTest;
  private Timer m_timer = new Timer();
  private double m_timeout;

  /**
   * Creates a new DriveDistance.
   *
   * @param inches The number of inches the robot will drive
   * @param speed The speed at which the robot will drive
   * @param drive The drive subsystem on which this command will run
   */
  public DriveDistance(double timeout, buttonCommandTest buttonCmdTest) {
    m_timer.reset();
    m_timeout = timeout;
    l_ButtonCommandTest = buttonCmdTest;
    addRequirements(l_ButtonCommandTest);
  }

  @Override
  public void initialize() {
    //Run once, at the start of the command
    m_timer.start();
  }

  @Override
  public void execute() {
    //run repeatedly, until isFinished() returns true
    l_ButtonCommandTest.buttonTest3();
    System.out.println(m_timer.get());
    
  }

  @Override
  public void end(boolean interrupted) {
    //Run once, at the end of the command
    m_timer.stop();
    m_timer.reset();
  }

  @Override
  public boolean isFinished() {
    // Determines when to finish the command
    return m_timer.get() >= m_timeout;
   
  }
}
