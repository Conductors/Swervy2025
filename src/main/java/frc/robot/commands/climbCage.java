package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.cageClimber;

public class climbCage extends Command {

  private boolean m_climbCage = false;
  private cageClimber l_CageClimber;
  private boolean m_upDirection = true;

  /**
   *Creates a new DriveDistance
   * @param speed The desired speed of claw wheels; positive draws algae IN, negative is OUT
   * @param ag The algaeGrabber subsystemt to control
*/
  public climbCage(boolean climbCage, boolean upDirection, cageClimber cc) {
    m_climbCage = climbCage;
    l_CageClimber = cc;
    m_upDirection = upDirection;
    addRequirements(l_CageClimber);
  }

  @Override
  public void initialize() {
    //Run once, at the start of the command

  }

  @Override
  public void execute() {
    //run repeatedly, until isFinished() returns true
    l_CageClimber.climb(m_climbCage, m_upDirection);
  }

  @Override
  public void end(boolean interrupted) {
    //Run once, at the end of the command
  }

  @Override
  public boolean isFinished() {
    // Determines when to finish the command
    return true;
  }
}
