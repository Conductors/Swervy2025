// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.setClawSpeed;

public class algaeGrabber extends SubsystemBase {
  /** Creates a new algaeGrabber. */
  private SparkMax wristMotor;  
  private SparkMax craneMotor;
  private SparkMax clawMotorUpper;
  private SparkMax clawMotorLower;
  
  private DutyCycleEncoder m_CraneEncoder;
  private DutyCycleEncoder m_WristEncoder;
  
  private double desiredCraneAngle = 0;
  private double desiredWristAngle = 0;
  private double actualCraneAngle = 0;
  private double actualWristAngle = 0;
  private double CraneAngleOffset = 0;
  private double WristAngleOffset = 0;
  private double CraneAngleStep = 0;
  private double WristAngleStep = 0;
  
  private final ProfiledPIDController m_CranePIDController;
  private final ProfiledPIDController m_WristPIDController;

  private double desiredClawSpeed = 0;

  /**
   * @param speed The speed at which the robot will drive
   * @param drive The drive subsystem on which this command will run
   */
  public algaeGrabber(int craneMotorPort,
                      int intakeMotorPortUpper,
                      int intakeMotorPortLower,
                      int wristMotorPort,
                      int craneEncoderPort,
                      int wristEncoderPort) {

    clawMotorUpper = new SparkMax(intakeMotorPortUpper, SparkLowLevel.MotorType.kBrushless);
    clawMotorLower = new SparkMax(intakeMotorPortLower, SparkLowLevel.MotorType.kBrushless);
    craneMotor = new SparkMax(craneMotorPort, SparkLowLevel.MotorType.kBrushless);
    wristMotor = new SparkMax(wristMotorPort, SparkLowLevel.MotorType.kBrushless);
    m_CraneEncoder = new DutyCycleEncoder(craneEncoderPort);
    m_CraneEncoder.setDutyCycleRange(1.0/1025.0, 1024.0/1025.0);

    m_WristEncoder = new DutyCycleEncoder(wristEncoderPort);
    m_WristEncoder.setDutyCycleRange(1.0/1025.0, 1024.0/1025.0);
    

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
    
    // Set the default command for a subsystem here. (set the claw speed to 0)
    setDefaultCommand(new setClawSpeed(0, this));
  }

  @Override
  public void periodic() {
    actualCraneAngle = (m_CraneEncoder.get()-Constants.aGConstants.k_CraneEncOffset)*2*Math.PI;
    actualWristAngle = (m_WristEncoder.get()-Constants.aGConstants.k_WristEncOffset)*2*Math.PI;

    //Publish Algae Grabber STuff to the Dashboard
    SmartDashboard.putNumber("CraneAngle", actualCraneAngle);
    SmartDashboard.putNumber("WristAngle", actualWristAngle);
    SmartDashboard.putNumber("ClawSpeed", desiredClawSpeed);
    
    //wristMotor.set(m_CranePIDController.calculate(actualCraneAngle, desiredCraneAngle));    //need to check motor direction
    //craneMotor.set(m_WristPIDController.calculate(actualWristAngle, desiredWristAngle));

    //clawMotorUpper.set(desiredClawSpeed);
    //clawMotorLower.set(desiredClawSpeed);

  }

  public void setDesiredCraneAngle(double angle) {
    desiredCraneAngle = angle + CraneAngleOffset;
  }

  public void setDesiredWristAngle(double angle) {
    desiredWristAngle = angle + WristAngleOffset;
  }

  public double getDesiredCraneAngle() {
    return desiredCraneAngle;
  }

  public double getDesiredWristAngle() {
    return desiredWristAngle;
  }
  
  public double getActualCraneAngle() {
    return actualCraneAngle;
  }

  public double getActualWristAngle() {
    return actualWristAngle;
  }

  public void setDesiredClawSpeed(double speed) {
    desiredClawSpeed = speed;
  }

  public double getDesiredClawSpeed() {
    return desiredClawSpeed;
  }

  public void IncCraneAngle() {
    CraneAngleOffset = CraneAngleOffset + CraneAngleStep;
    }

  public void DecCraneAngle() {
    CraneAngleOffset = CraneAngleOffset - CraneAngleStep;
    }
  
  public void IncWristAngle() {
    CraneAngleOffset = CraneAngleOffset + CraneAngleStep;
    }
    
  public void DecWristAngle() {
    CraneAngleOffset = CraneAngleOffset - CraneAngleStep;
    }

}