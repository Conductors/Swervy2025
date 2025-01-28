package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Position;
import frc.robot.subsystems.algaeGrabber;

public class setCranePosition extends Command {

  private Position m_setPoint = Constants.Position.keStow;
  private algaeGrabber lAlgaeGrabber;

  /**
   *Creates a new DriveDistance
   * @param setPoint The desired setpoint for the Algae Grabber subsystem
   * @param ag The algaeGrabber subsystemt to control
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
      case keProcessor:
      case keReef2:
      case keReef3:
      case keGround:
        lAlgaeGrabber.setDesiredCraneAngle(Constants.aGConstants.k_CraneAngleSetpoint[m_setPoint.ordinal()]);
        lAlgaeGrabber.setDesiredWristAngle(Constants.aGConstants.k_WristAngleSetpoint[m_setPoint.ordinal()]);
        break;
      case keReef1:
      case keReef4:
      default:
        System.out.println("this is not a valid algae setpoint");
        break;
    }
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
