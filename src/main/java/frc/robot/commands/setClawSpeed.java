package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algaeGrabber;

public class setClawSpeed extends Command {

  private double m_ClawSpeed = 0;
  private algaeGrabber lAlgaeGrabber;

  /**
   *Creates a new DriveDistance
   * @param speed The desired speed of claw wheels; positive draws algae IN, negative is OUT
   * @param ag The algaeGrabber subsystemt to control
*/
  public setClawSpeed(double speed, algaeGrabber ag) {
    m_ClawSpeed = speed;
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
    lAlgaeGrabber.setDesiredClawSpeed(m_ClawSpeed);
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
