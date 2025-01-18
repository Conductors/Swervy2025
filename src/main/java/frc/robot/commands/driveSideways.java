package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Drivetrain;

public class driveSideways extends Command{
private Drivetrain lDrivetrain;
private double m_distance = 0;
private double m_currDistance = 0;
private double m_initialPos = 0;
private double m_currentPos = 0;
private double m_Period = 0;
private double m_direction = 0;

  /**
   *Creates a new DriveDistance
   * @param meters
   * @param drivetrain
*/
  public driveSideways(double p_Distance, double p_Period, Drivetrain driveTrain) {
    m_direction = Math.signum(p_Distance);
    m_distance = Math.abs(p_Distance);
    lDrivetrain = driveTrain;
    m_Period = p_Period;
    addRequirements(lDrivetrain);
  }

  @Override
  public void initialize() {
    //Run once, at the start of the command
    m_initialPos = lDrivetrain.m_odometry.getPoseMeters().getY();

  }

  @Override
  public void execute() {
    //run repeatedly, until isFinished() returns true
    m_currentPos = lDrivetrain.m_odometry.getPoseMeters().getY();
    m_currDistance = Math.abs(m_currentPos - m_initialPos);
    lDrivetrain.drive(0,m_direction*Constants.AutoConstants.kMaxSpeedMetersPerSecond, 0, false, m_Period);

  }

  @Override
  public void end(boolean interrupted) {
    //Run once, at the end of the command
    lDrivetrain.drive(0, 0, 0, false, m_Period);

  }

  @Override
  public boolean isFinished() {
    // Determines when to finish the command
    return m_currDistance >= m_distance;
   
  }
}
