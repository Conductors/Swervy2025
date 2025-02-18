package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Position;
import frc.robot.subsystems.coralSubsystem;

public class setCoralHeight extends Command {

  private Position m_setPoint = Constants.Position.keStow;
  private coralSubsystem m_CoralSubsystem;

  /**
   *Creates a new DriveDistance
   * @param setPoint The desired setpoint for the Coral Elevator + Tilt subsystem
   * @param cs The coralSubsystem subsystem to control
  */
  public setCoralHeight(Position setPoint, coralSubsystem cs) {
    m_setPoint = setPoint;
    m_CoralSubsystem = cs;
    addRequirements(m_CoralSubsystem);
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
      case keReef1:
      case keReef4:
        m_CoralSubsystem.setDesiredHeight(Constants.csConstants.k_ElevatorHeight[m_setPoint.ordinal()]);        
        break;
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
