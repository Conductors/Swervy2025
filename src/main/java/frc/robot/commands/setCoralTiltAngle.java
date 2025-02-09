package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.tiltPosition;
import frc.robot.subsystems.coralSubsystem;

public class setCoralTiltAngle extends Command {

  private tiltPosition m_tiltPosition = Constants.tiltPosition.keStow;
  private coralSubsystem m_CoralSubsystem;

  /**
   *Creates a new DriveDistance
   * @param setPoint The desired setpoint for the Coral Elevator + Tilt subsystem
   * @param cs The coralSubsystem subsystem to control
  */
  public setCoralTiltAngle(tiltPosition setPoint, coralSubsystem cs) {
    m_tiltPosition = setPoint;
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
    switch(m_tiltPosition) {
      case keStow:
      case keLoad:
      case keScore:
        m_CoralSubsystem.setDesiredTiltAngle(Constants.csConstants.k_tiltAngleSetpoint[m_tiltPosition.ordinal()]);        
        break;
      default:
        System.out.println("this is not a valid coral tilt setpoint");
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
