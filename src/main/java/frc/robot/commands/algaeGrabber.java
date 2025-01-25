// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.buttonCommandTest;

public class algaeGrabber extends SubsystemBase {
  /** Creates a new algaeGrabber. */
  private SparkMax clawMotor;  
  private SparkMax craneMotor;
  private double clawSpeed = 0;
  private double desiredCraneAngle = 0;
  private double desiredWristAngle = 0;
  private final ProfiledPIDController m_CranePIDController;
  private final ProfiledPIDController m_WristPIDController;


  /**
   * @param speed The speed at which the robot will drive
   * @param drive The drive subsystem on which this command will run
   */
  public algaeGrabber(int craneMotorPort,
                      int intakeMotorPort) {

  clawMotor = new SparkMax(intakeMotorPort, SparkLowLevel.MotorType.kBrushless);
  craneMotor = new SparkMax(craneMotorPort, SparkLowLevel.MotorType.kBrushless);

      m_CranePIDController =
      new ProfiledPIDController(
          Constants.KpCrane,
          0,
          0,
          new TrapezoidProfile.Constraints(
              Constants.CraneMaxVelocity,
              Constants.CraneMaxAccel));
    m_CranePIDController.setTolerance(.05);  //sets the tolerance for the PID controller, in meters

    m_WristPIDController =
    new ProfiledPIDController(
        Constants.KpWrist,
        0,
        0,
        new TrapezoidProfile.Constraints(
            Constants.WristMaxVelocity, 
            Constants.WristMaxAccel));
  m_WristPIDController.setTolerance(.05);  //sets the tolerance for the PID controller, in meters
  }

  }

  @Override
  public void periodic() {
   
  }
  public void setDesiredCraneAngle(double angle) {
    desiredCraneAngle = angle;
  }
  public void setDesiredWristAngle(double angle) {
    desiredWristAngle = angle;
  }
  public double getDesiredCraneAngle() {
    return desiredCraneAngle;
  }
  public double getDesiredWristAngle() {
    return desiredWristAngle;
  }
}