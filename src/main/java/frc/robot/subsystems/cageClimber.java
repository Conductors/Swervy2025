// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class cageClimber extends SubsystemBase {
  /** Creates a new algaeGrabber. */
  private SparkMax cageWinch;    
  private int cageWinchPort = 30;
  private double winchSpeed = 0;

  public cageClimber() {

    //Setup Cage Winch motor as a sparkmax with braking idle mode
    cageWinch = new SparkMax(cageWinchPort, SparkLowLevel.MotorType.kBrushless);
    SparkMaxConfig cageConfig = new SparkMaxConfig();
      cageConfig.idleMode(IdleMode.kBrake);
    cageWinch.configure(cageConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    //setDefaultCommand(new climbCage(false, this));
  }

  @Override
  public void periodic() {
    //periodic stuff for cageClimber
    cageWinch.set(winchSpeed);
    SmartDashboard.putNumber("winchSpeed", winchSpeed);

    //Publish Stuff to Dashboard
    SmartDashboard.putNumber("Winch Speed", winchSpeed);

  }

  public void climb(boolean climbing, boolean isUp) {
    if(climbing) {
      if(isUp)
      { winchSpeed = -5; } 
      else 
      { winchSpeed = .5; }
    }
    else 
    {
      winchSpeed = 0;
    }
  }
 

}