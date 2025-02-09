package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralSubsystem;

public class setGateState extends Command {

  private boolean m_gateOpen = false;
  private coralSubsystem m_CoralSubsystem;

  /**
   *Creates a new DriveDistance
   * @param setPoint The desired setpoint for the Coral Elevator + Tilt subsystem
   * @param cs The coralSubsystem subsystem to control
  */
  public setGateState(boolean isOpen, coralSubsystem cs) {
    m_gateOpen = isOpen;
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
    if(m_gateOpen) {
      m_CoralSubsystem.openGate();
    } else {
      m_CoralSubsystem.closeGate();
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
