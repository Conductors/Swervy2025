package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Drivetrain;

public class driveStraightPID extends Command{
private Drivetrain lDrivetrain;
private double m_distance = 0;
private double m_currDistance = 0;
private double m_goalPos = 0;
private double m_initialPos = 0;
private double m_currentPos = 0;
private double m_Period = 0;
private double m_direction = 0;
private final ProfiledPIDController m_PIDController;

  /**
   *Creates a new DriveDistance
   * @param meters
   * @param drivetrain
*/
  public driveStraightPID(double p_Distance, double p_Period, Drivetrain driveTrain) {
    m_direction = Math.signum(p_Distance);
    m_distance = p_Distance;
    lDrivetrain = driveTrain;
    m_Period = p_Period;
    addRequirements(lDrivetrain);

      //Gains specific to controlling via 'drive' the robot to a position specified by the user
    m_PIDController =
      new ProfiledPIDController(
          1,
          0,
          0,
          new TrapezoidProfile.Constraints(
              Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond, 
              Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared));
    m_PIDController.setTolerance(.05);  //sets the tolerance for the PID controller, in meters
  }

  @Override
  public void initialize() {
    //Run once, at the start of the command
    m_initialPos = lDrivetrain.m_odometry.getPoseMeters().getX();
    m_goalPos = m_initialPos + m_distance;

  }

  @Override
  public void execute() {
    //run repeatedly, until isFinished() returns true
    m_currentPos = lDrivetrain.m_odometry.getPoseMeters().getX();
    
    //Drive the robot to the goal position, clamping the error output to +/- max speed
    lDrivetrain.drive(
      MathUtil.clamp(m_PIDController.calculate(m_currentPos, m_goalPos), 
        -Constants.AutoConstants.kMaxSpeedMetersPerSecond, 
        Constants.AutoConstants.kMaxSpeedMetersPerSecond),
      0, 
      0, 
      false, 
      m_Period);
  }

  @Override
  public void end(boolean interrupted) {
    //Run once, at the end of the command
    lDrivetrain.drive(0, 0, 0, false, m_Period);

  }

  @Override
  public boolean isFinished() {
    // Determines when to finish the command
    //return m_currDistance >= m_distance;
    return m_PIDController.atGoal();
   
  }
}
