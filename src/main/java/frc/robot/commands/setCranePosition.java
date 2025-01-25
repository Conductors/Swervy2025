package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Drivetrain;
import frc.robot.Constants.Position;

public class setCranePosition extends Command{

  private Position m_setPoint = Constants.Position.keStow;
  private algaeGrabber lAlgaeGrabber;

  /**
   *Creates a new DriveDistance
   * @param meters
   * @param drivetrain
*/
  public setCranePosition(Position setPoint, algaeGrabber ag) {
    m_setPoint = setPoint;
    lAlgaeGrabber = ag;
    addRequirements(lAlgaeGrabber);
  }

  @Override
  public void initialize() {
    //Run once, at the start of the command

  }

  @Override
  public void execute() {
    //run repeatedly, until isFinished() returns true
    switch(m_setPoint) {
      case keStow:
        lAlgaeGrabber.setDesiredCraneAngle(0);
        lAlgaeGrabber.setDesiredWristAngle(0);
        break;
      default:
        break;
    }
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
