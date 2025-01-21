package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Drivetrain;

public class driveSpinways extends Command{
private Drivetrain lDrivetrain;
private double m_distance = 0;
private double m_currDistance = 0;
private double m_initialPos = 0;
private double m_currentPos = 0;
private double m_Period = Constants.kDefaultPeriod;
private double m_direction = 0;

  /**
   *Creates a new command which will spin the robot in place a given angle.  This function optimizes to minimize the
   * angle travelled to between - pi and pi
   * @param angle in Radians, positive is counter clockwise; 
   * @param drivetrain
*/
  public driveSpinways(double p_Angle, double p_Period, Drivetrain driveTrain) {
    double wrappedAngle = MathUtil.angleModulus(p_Angle); //Wrap the angle to be between -pi and pi
    m_direction = Math.signum(wrappedAngle);
    m_distance = Math.abs(wrappedAngle);
    lDrivetrain = driveTrain;
    m_Period = p_Period;
    addRequirements(lDrivetrain);
  }

  @Override
  public void initialize() {
    //Run once, at the start of the command
    m_initialPos = lDrivetrain.m_odometry.getPoseMeters().getRotation().getRadians();

  }

  @Override
  public void execute() {
    //run repeatedly, until isFinished() returns true
    m_currentPos = lDrivetrain.m_odometry.getPoseMeters().getRotation().getRadians();
    m_currDistance = Math.abs(m_currentPos - m_initialPos);
    lDrivetrain.drive(0,0,m_direction*Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond, false, m_Period);

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
