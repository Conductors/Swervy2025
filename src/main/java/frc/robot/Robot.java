// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.setCranePosition;
import frc.robot.commands.setGateState;
import frc.robot.Constants.Position;
import frc.robot.Constants.tiltPosition;
import frc.robot.commands.climbCage;
import frc.robot.commands.driveSidewaysPID;
import frc.robot.commands.driveSpinwaysPID;
import frc.robot.commands.driveStraightPID;
import frc.robot.commands.driveToPositionPID;
import frc.robot.commands.setClawSpeed;
import frc.robot.commands.setCoralHeight;
import frc.robot.commands.setCoralState;
import frc.robot.commands.setCoralTiltAngle;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.algaeGrabber;
import frc.robot.subsystems.cageClimber;
import frc.robot.subsystems.coralSubsystem;

public class Robot extends TimedRobot {
  private final CommandXboxController m_controller = new CommandXboxController(0);
  private final CommandXboxController m_controller2 = new CommandXboxController(1);
  private Trigger yButton     = m_controller.y(); 
  private Trigger xButton     = m_controller.x(); 
  private Trigger aButton     = m_controller.a(); 
  private Trigger bButton     = m_controller.b(); 
  private Trigger startButton = m_controller.start(); 
  private Trigger backButton  = m_controller.back();
  private Trigger lbButton     = m_controller.leftBumper();
  private Trigger rbButton    = m_controller.rightBumper(); 
  private Trigger lBTrigger   = m_controller.leftTrigger(0.1); 
  private Trigger rBTrigger   = m_controller.rightTrigger(.1); 
  private Trigger povUp       = m_controller.povUp();
  private Trigger povDown     = m_controller.povDown();
  private Trigger povLeft     = m_controller.povLeft();
  private Trigger povRight    = m_controller.povRight();

  private Trigger yButton2    = m_controller2.y(); 
  private Trigger xButton2    = m_controller2.x(); 
  private Trigger aButton2    = m_controller2.a(); 
  private Trigger bButton2    = m_controller2.b(); 
  private Trigger startButton2= m_controller2.start(); 
  private Trigger backButton2 = m_controller2.back(); 
  private Trigger lbButton2   = m_controller2.leftBumper(); 
  private Trigger rbButton2   = m_controller2.rightBumper(); 
  private Trigger lBTrigger2  = m_controller2.leftTrigger(0.1); 
  private Trigger rBTrigger2  = m_controller2.rightTrigger(.1); 
  private Trigger povUp2      = m_controller2.povUp();
  private Trigger povDown2    = m_controller2.povDown();
  private Trigger povLeft2    = m_controller2.povLeft();
  private Trigger povRight2   = m_controller2.povRight();

  
  private boolean isHighGear = false;
  private boolean isFieldRelative = false;
  private boolean isAlgaeRelative = false;

  private final Drivetrain m_swerve = new Drivetrain();
  private final Field2d m_field = new Field2d();

  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();

  private algaeGrabber m_AlgaeGrabber = new algaeGrabber(Constants.aGConstants.k_CraneMotorPort,
                                                          Constants.aGConstants.k_ClawMotorPortUpper,
                                                          Constants.aGConstants.k_ClawMotorPortLower,
                                                          Constants.aGConstants.k_WristMotorPort,
                                                          Constants.aGConstants.k_CraneEncPort,
                                                          Constants.aGConstants.k_WristEncPort );

  private coralSubsystem m_CoralSubsystem = new coralSubsystem();
  private cageClimber m_CageClimber = new cageClimber();
  

  // Slew rate limiters to make joystick inputs more gentle; Passing in "3" means 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private Command m_autonomousCommand;

  private String m_autoSelected;
  private final SendableChooser<String> m_AutoChooser = new SendableChooser<>();

  private final LEDSubsystem ledSystem = new LEDSubsystem();


public Robot() {
  CameraServer.startAutomaticCapture();
}

  @Override
  public void robotInit() {
    m_AutoChooser.setDefaultOption("None", Constants.AutoConstants.kAutoProgram[0]);
    m_AutoChooser.addOption("Auto 1", Constants.AutoConstants.kAutoProgram[1]);
    m_AutoChooser.addOption("Auto 2", Constants.AutoConstants.kAutoProgram[2]);
    m_AutoChooser.addOption("Auto 3", Constants.AutoConstants.kAutoProgram[3]);
    m_AutoChooser.addOption("BackUp", Constants.AutoConstants.kAutoProgram[4]);
    m_AutoChooser.addOption("ScoreOneCoral", Constants.AutoConstants.kAutoProgram[5]);
    m_AutoChooser.addOption("ScoreCoralClearAlgae", Constants.AutoConstants.kAutoProgram[6]);
    m_AutoChooser.addOption("ScoreCoralTake1Algae", Constants.AutoConstants.kAutoProgram[7]);

    SmartDashboard.putData("Auto Choices", m_AutoChooser);  //Sync the Autochooser

  
  }

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    SmartDashboard.putData("Auto Choices", m_AutoChooser);  //Sync the Autochooser
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    
    publisher.set(m_swerve.m_odometry.getPoseMeters());
  }


  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    SmartDashboard.putData("Field", m_field);

    m_autoSelected = m_AutoChooser.getSelected();
    m_autonomousCommand = getAutonomousCommand();
    
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }    
  }

  @Override
  public void teleopInit() {
    // Do this in either robot or subsystem init
    SmartDashboard.putData("Field", m_field);
    publisher.set(m_swerve.m_odometry.getPoseMeters());
    
    // This makes sure that the autonomous stops running when teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    /* Button Triggers */
    backButton.onTrue(shiftGears()); 
    startButton.onTrue(changeIsFieldRelative());
    
    aButton.onTrue(new setCranePosition(Constants.Position.keStow, m_AlgaeGrabber));
    bButton.onTrue(new setCranePosition(Constants.Position.keProcessor, m_AlgaeGrabber));
    yButton.onTrue(new setCranePosition(Constants.Position.keReef3, m_AlgaeGrabber));
    xButton.onTrue(new setCranePosition(Constants.Position.keReef2, m_AlgaeGrabber));
    
    lBTrigger.whileTrue(new setClawSpeed(0.5, m_AlgaeGrabber))
              .onFalse(new setClawSpeed(0, m_AlgaeGrabber));
    rBTrigger.whileTrue(new setClawSpeed(-m_controller.getRightTriggerAxis(), m_AlgaeGrabber))
              .onFalse(new setClawSpeed(0, m_AlgaeGrabber));    //check - out is negative
    lbButton.whileTrue(new setClawSpeed(0.5, m_AlgaeGrabber))
              .onFalse(new setClawSpeed(0, m_AlgaeGrabber));
    //rbButton.whileTrue(new setClawSpeed(-0.5, m_AlgaeGrabber))
    //          .onFalse(new setClawSpeed(0, m_AlgaeGrabber));
    rbButton.onTrue(changeIsAlgaeRelative());

    lbButton2.onTrue(new setGateState(true, m_CoralSubsystem));
    lbButton2.onFalse(new setGateState(false, m_CoralSubsystem));
    
    aButton2.onTrue(Commands.sequence(new setCoralHeight(Constants.Position.keReef1, m_CoralSubsystem),
                                      new setCoralTiltAngle(Constants.tiltPosition.keScore, m_CoralSubsystem)));    
    
    bButton2.onTrue(Commands.sequence(new setCoralTiltAngle(Constants.tiltPosition.keScore, m_CoralSubsystem),
                                      new setCoralHeight(Constants.Position.keReef3, m_CoralSubsystem)));
    
    yButton2.onTrue(Commands.sequence(new setCoralTiltAngle(Constants.tiltPosition.keScore, m_CoralSubsystem),
                                      new setCoralHeight(Constants.Position.keReef4, m_CoralSubsystem)));
      
    xButton2.onTrue(Commands.sequence(new setCoralTiltAngle(Constants.tiltPosition.keScore, m_CoralSubsystem),
                                      new setCoralHeight(Constants.Position.keReef2, m_CoralSubsystem)));
        
    backButton2.onTrue(Commands.sequence(new setCoralTiltAngle(Constants.tiltPosition.keStow, m_CoralSubsystem),
                                        new setCoralHeight(Constants.Position.keStow, m_CoralSubsystem)));
    
    startButton2.onTrue(Commands.sequence(new setCoralTiltAngle(Constants.tiltPosition.keLoad, m_CoralSubsystem),
                                      new setCoralHeight(Constants.Position.keCoralStation, m_CoralSubsystem)));
        
    rbButton2.whileTrue(new climbCage(true, true, m_CageClimber))
              .onFalse(new climbCage(false, true, m_CageClimber));
    rBTrigger2.whileTrue(new climbCage(true, false, m_CageClimber))
              .onFalse(new climbCage(false, false, m_CageClimber));

    povUp.onTrue(new InstantCommand(() -> m_AlgaeGrabber.IncCraneAngle()));
    povDown.onTrue(new InstantCommand(() -> m_AlgaeGrabber.DecCraneAngle()));
    povRight.onTrue(new InstantCommand(() -> m_AlgaeGrabber.IncWristAngle()));
    povLeft.onTrue(new InstantCommand(() -> m_AlgaeGrabber.DecWristAngle()));
    
    povUp2.onTrue(new InstantCommand(() -> m_CoralSubsystem.incHeightOffset()));
    povDown2.onTrue(new InstantCommand(() -> m_CoralSubsystem.decHeightOffset()));
    povRight2.onTrue(new InstantCommand(() -> m_CoralSubsystem.incTiltOffset()));
    povLeft2.onTrue(new InstantCommand(() -> m_CoralSubsystem.decTiltOffset()));
  }
  
  @Override
  public void autonomousPeriodic() {
    //driveWithJoystick(false);
    publishToDashboard();
    m_swerve.publishToDashboard();
    m_swerve.updateOdometry();

    // Do this in either robot periodic or subsystem periodic
    m_field.setRobotPose(m_swerve.m_odometry.getPoseMeters());
  }

  @Override
  public void teleopPeriodic() {
    publishToDashboard();
    //m_swerve.publishToDashboard();
    //switchGears(false);
    m_swerve.updateOdometry();

    // Do this in either robot periodic or subsystem periodic
    m_field.setRobotPose(m_swerve.m_odometry.getPoseMeters());
 
    //Set the max speed constant to use high (regular) or low speed based on isHighGear
    double l_MaxSpeed = isHighGear?Constants.kMaxRobotSpeed:Constants.kMaxRobotSpeedLowGear;
    
    if(!isAlgaeRelative)
    {
      final var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.1))
        * l_MaxSpeed;
      SmartDashboard.putNumber("xSpeed", xSpeed);

      final var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.1))
          * l_MaxSpeed;
      SmartDashboard.putNumber("ySpeed", ySpeed);

      final var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.1))
          * l_MaxSpeed;
      SmartDashboard.putNumber("rot", rot);

      m_swerve.drive(xSpeed, ySpeed, rot, isFieldRelative, getPeriod());    
    } else {  //This IS algae relative.  
      {
        final var xSpeed =
          -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.1))
          * l_MaxSpeed;
        SmartDashboard.putNumber("xSpeed", xSpeed);
  
        final var ySpeed =
          -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.1))
            * l_MaxSpeed;
        SmartDashboard.putNumber("ySpeed", ySpeed);
  
        final var rot =
          -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.1))
            * l_MaxSpeed;
        SmartDashboard.putNumber("rot", rot);
  
        m_swerve.drive(xSpeed, ySpeed, rot, isFieldRelative, getPeriod()); 
    }

  }

  }


  public void publishToDashboard()
  {
    SmartDashboard.putNumber("Controller Left X", m_controller.getLeftX());
    SmartDashboard.putNumber("Controller Left Y", m_controller.getLeftY());
    SmartDashboard.putNumber("Controller Right X", m_controller.getRightX());
    SmartDashboard.putNumber("Gyro Angle", m_swerve.m_gyro.getRotation2d().getDegrees());
    SmartDashboard.putBoolean("High Gear Enabled", isHighGear);
    SmartDashboard.putBoolean("isFieldRelative", isFieldRelative);
    SmartDashboard.putBoolean("isAlgaeRelative", isAlgaeRelative);

    SmartDashboard.putNumber("Crane Angle", m_AlgaeGrabber.getActualCraneAngle());
    SmartDashboard.putNumber("Wrist Angle", m_AlgaeGrabber.getActualWristAngle());
    SmartDashboard.putNumber("Tilt Angle", m_CoralSubsystem.getActualTiltAngle());

    SmartDashboard.putNumber("RightTrigger", m_controller.getRightTriggerAxis());
    
  }


  /* AUTO Stuff below here */
  public Command getAutonomousCommand() {

      Command temp = new Command() {};
    // Grabs the choser Auto from Shuffleboard
    switch (m_autoSelected) {
      case "None":
      temp = m_swerve.getPathPlannerCommand();
        break;
      case "Auto 1":
        temp = Commands.sequence(
          new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))),
          driveStraight(1));
        break;
      case "Auto 2":
        temp = Commands.sequence(
          new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))),
          new InstantCommand(() -> System.out.println("Command 1:")),
          driveStraight(1),
          new InstantCommand(() -> System.out.println("Stop & wait  .5 seconds")),
          new InstantCommand(() -> m_swerve.drive(0,0,0,false, getPeriod())).repeatedly().withTimeout(.5),
          new InstantCommand(() -> System.out.println("Done !")));
        break;
      case "Auto 3":
        temp = Commands.sequence(
        new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))),
        new InstantCommand(() -> System.out.println("Command 1:")),
        driveToPosition(new Pose2d(0, 0, new Rotation2d(Math.PI/2))),
        new InstantCommand(() -> System.out.println("Stop & wait  .5 seconds")),
        new InstantCommand(() -> m_swerve.drive(0,0,0,false, getPeriod())).repeatedly().withTimeout(.5),
        new InstantCommand(() -> System.out.println("Done !")));
        break;
      case "BackUp":
        temp = Commands.sequence(
          new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))),
          new InstantCommand(() -> System.out.println("Command 1:")),
          driveStraight(-1),
          new InstantCommand(() -> System.out.println("Stop & wait  .5 seconds")),
          new InstantCommand(() -> m_swerve.drive(0,0,0,false, getPeriod())).repeatedly().withTimeout(.5),
          new InstantCommand(() -> System.out.println("Done !")));
          break;
        case "ScoreOneCoral":
          temp = Commands.sequence(
            new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))),
            new InstantCommand(() -> System.out.println("Drive Forward")),
            driveStraight(1.5),
            //new InstantCommand(() -> System.out.println("Set Coral Height - Reef 1")),
            //setCoralHeight(Constants.Position.keReef2),
            //new InstantCommand(() -> System.out.println("Set Coral Tilt - Score")),
            new WaitCommand(1),
            scoreCoralReef2(),
            new InstantCommand(() -> System.out.println("Wait")),
            new WaitCommand(2),
            new InstantCommand(() ->  new setGateState(true, m_CoralSubsystem)),
            new WaitCommand(2),
            new InstantCommand(() -> System.out.println("Set Coral Height - Stow")),
            stowCoral(),
            new InstantCommand(() ->  new setGateState(false, m_CoralSubsystem)),
            new InstantCommand(() -> System.out.println("Stop & wait  .5 seconds")),
            new InstantCommand(() -> m_swerve.drive(0,0,0,false, getPeriod())).repeatedly().withTimeout(.5),
            new InstantCommand(() -> System.out.println("Done !")));
            break;
      case "ScoreCoralTake1Algae":
      temp = Commands.sequence(
        new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))),
        new InstantCommand(() -> System.out.println("Drive Forward")),
        driveStraight(1.25),
        new InstantCommand(() -> System.out.println("Set Coral Height - Reef 1")),
        setCoralHeight(Constants.Position.keReef1),
        new InstantCommand(() -> System.out.println("Set Coral Tilt - Score")),
        scoreCoralReef1(),
        new InstantCommand(() -> System.out.println("Wait")),
        new WaitCommand(3.5),
        new InstantCommand(() -> System.out.println("Go Backwards")),
        driveStraight(-0.5),
        new InstantCommand(() -> System.out.println("Rotate 90 degrees")),
        driveSpinways(-Math.PI/2),
        new InstantCommand(() -> System.out.println("Set Algae Height - Reef 2")),
        setAlgaeHeight(Constants.Position.keReef2),
        new InstantCommand(() -> System.out.println("Drive Forward and set Claw to Intake Speed")),
        driveStraight(0.5).alongWith(clawSpeedScore()),
        new InstantCommand(() -> System.out.println("Wait")),
        new WaitCommand(3.5),
        new InstantCommand(() -> System.out.println("Go Backwards and Stop Claw")),
        driveStraight(-0.5).alongWith(clawSpeedZero()),
        new InstantCommand(() -> System.out.println("Go left")),
        driveSideways(1),
        new InstantCommand(() -> System.out.println("Throw out Algae")),
        clawSpeedOutput(),
        new InstantCommand(() -> System.out.println("Stop Claw")),
        clawSpeedZero(),
        new InstantCommand(() -> System.out.println("Go right")),
        driveSideways(-1),
        new InstantCommand(() -> System.out.println("Stop & wait  .5 seconds")),
        new InstantCommand(() -> m_swerve.drive(0,0,0,false, getPeriod())).repeatedly().withTimeout(.5),
        new InstantCommand(() -> System.out.println("Done !")));
          break;
      case "diagScoreReef":
        diagScoreAuto(true);
        break;
      case "ScoreCoralClearAlgae":
        temp = Commands.sequence(
            new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))),
            new InstantCommand(() -> System.out.println("Drive Forward")),
            driveStraight(1.25),
            scoreCoral(Constants.Position.keReef1),
            driveStraight(-0.5)
        );
        break;
      default:
        break;
    }
    return temp;
  }
 
  public InstantCommand resetOdoCommand() {
    return new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0))));
  }
  public Command driveStraight(double dist) {
    return new driveStraightPID(dist, getPeriod(), m_swerve);
  }
    
  public Command driveSideways(double dist) {
    return new driveSidewaysPID(dist, getPeriod(), m_swerve);
  }

  public Command driveSpinways(double angle) {
    return new driveSpinwaysPID(angle, getPeriod(), m_swerve);
  }

  public Command driveToPosition(Pose2d position) {
    return new driveToPositionPID(position, getPeriod(), m_swerve);
  }

  public Command setCoralHeight(Position pos) {
    return new setCoralHeight(pos, m_CoralSubsystem);
  }

  public Command scoreCoralReef1() {
    return new setCoralState(Constants.Position.keReef1, Constants.tiltPosition.keScore, m_CoralSubsystem);
  }

  public Command scoreCoralReef2() {
    return new setCoralState(Constants.Position.keReef2, Constants.tiltPosition.keScore, m_CoralSubsystem);
  }

  public Command stowCoral() {
    return new setCoralHeight(Constants.Position.keStow, m_CoralSubsystem);
  }

  public Command setAlgaeHeight(Position pos) {
    return new setCranePosition(pos, m_AlgaeGrabber);
  }

public Command clawSpeedScore() {
  return new setClawSpeed(3, m_AlgaeGrabber);
}

public Command clawSpeedZero() {
  return new setClawSpeed(0, m_AlgaeGrabber);
}

public Command clawSpeedOutput() {
  return new setClawSpeed(-3, m_AlgaeGrabber);
}

public Command Elbethle(tiltPosition tiltPos) {
  return new setCoralTiltAngle(tiltPos, m_CoralSubsystem);
}


public Command scoreCoral(Position pos) {
  return Commands.sequence(
  new InstantCommand(() -> System.out.println("Set Coral Height - Reef 1")),
  new setCoralHeight(pos, m_CoralSubsystem),
  new InstantCommand(() -> System.out.println("Set Coral Tilt - Score")),
  Elbethle(Constants.tiltPosition.keScore),
  new InstantCommand(() -> System.out.println("Wait")),
  new WaitCommand(3.5));
}

public Command takeAlgae(Position algaePos) {
  return Commands.sequence(
    new InstantCommand(() -> System.out.println("Set Algae Height - Reef 2")),
        setAlgaeHeight(Constants.Position.keReef2),
        new InstantCommand(() -> System.out.println("Drive Forward and set Claw to Intake Speed")),
        driveStraight(0.5).alongWith(clawSpeedScore()),
        new InstantCommand(() -> System.out.println("Wait")),
        new WaitCommand(3.5),
        clawSpeedZero()
  );
}

  public Command shiftGears() {
    return Commands.sequence(
        new InstantCommand(() -> isHighGear=!isHighGear)
    );
  }

  public Command changeIsFieldRelative() {
    return Commands.sequence(
        new InstantCommand(() -> isFieldRelative=!isFieldRelative)
    );
  }

  public Command changeIsAlgaeRelative() {
    return Commands.sequence(
        new InstantCommand(() -> isAlgaeRelative=!isAlgaeRelative)
    );
  }

/**
 * diagScoreAuto starts in front of the cage closest to the wall facing towards the driver stations
 * Drives towards the reef, scores on Reef 1, then backs up
 * @param isOnBlueSide Set to TRUE if the Robot starts on the BlueSide (vs. false for RedSide)
 * @return
 */
public Command diagScoreAuto(boolean isOnBlueSide) {
  return Commands.sequence(
    new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))),
    new InstantCommand(() -> System.out.println("Drive Forward")),
    driveStraight(1.25), 
    new InstantCommand(() -> System.out.println("Turn 60 Degrees CCW")),
    driveSpinways(Math.PI/3),
    new InstantCommand(() -> System.out.println("Drive Forward to Reef")),
    driveStraight(3.3), //calculate the diagnoal distance to the reef
    new InstantCommand(() -> System.out.println("Set Coral Height - Reef 1")),
    setCoralHeight(Constants.Position.keReef1),
    new InstantCommand(() -> System.out.println("Set Coral Tilt - Score")),
    scoreCoralReef1(),
    new InstantCommand(() -> System.out.println("Wait")),
    new WaitCommand(3.5),
    new InstantCommand(() -> System.out.println("Set Coral Height - Stow")),
    stowCoral(),
    new InstantCommand(() -> System.out.println("Stop & wait  .5 seconds")),
    new InstantCommand(() -> m_swerve.drive(0,0,0,false, getPeriod())).repeatedly().withTimeout(.5),
    new InstantCommand(() -> System.out.println("Done !")));
}
  public void oldTrajectorySTuff() {
    /*
    //delete.  This was the Trajectory stuff in the 'getAuto' function for a while, until PathPlanner implementation
        // Create config for trajectory
        TrajectoryConfig fwdconfig = new TrajectoryConfig(
          AutoConstants.kMaxSpeedMetersPerSecond,
          AutoConstants.kMaxAccelerationMetersPerSecondSquared)
          .setKinematics(m_swerve.m_kinematics);  // Add kinematics to ensure max speed is actually obeyed
      
      // Create config for trajectory
      TrajectoryConfig backconfig = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(m_swerve.m_kinematics)  // Add kinematics to ensure max speed is actually obeyed
            .setReversed(true);           // "Reversed" will allow the robot to go backwards through trajs
  
      // An example trajectory to follow. All units in meters.
      Trajectory fwdTraj = TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          new Pose2d(0, 0, new Rotation2d(0)),
          // Pass through these two interior waypoints, making an 's' curve path
          List.of(new Translation2d(.75, 0)),
          // End 1.5 meters straight ahead of where we started, facing forward
          new Pose2d(1.5, 0, new Rotation2d(0)),
          fwdconfig);
  
       Trajectory backTraj = TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          new Pose2d(0, 0, new Rotation2d(0)),
          // Pass through these two interior waypoints, making an 's' curve path
          List.of(new Translation2d(-.10, 0)),
          // End 1.5 meters straight ahead of where we started, facing forward
          new Pose2d(-1.5, 0, new Rotation2d(0)),
          backconfig);
  
      Trajectory leftTraj = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(0, .75)),
        // End 1.5 meters straight ahead of where we started, facing forward
        new Pose2d(0,1.5, new Rotation2d(0)),
        backconfig);
  
      Trajectory conTrajectory = fwdTraj.concatenate(backTraj);
  
      var thetaController = new ProfiledPIDController(
          AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
  
      SwerveControllerCommand swerveControllerCommand1 = 
      new SwerveControllerCommand(
        fwdTraj,
        m_swerve::getPose,
        m_swerve.m_kinematics,
        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_swerve::setModuleStates,
        m_swerve);
  
      SwerveControllerCommand swerveControllerCommand2 = 
      new SwerveControllerCommand(
        backTraj,
        m_swerve::getPose,
        m_swerve.m_kinematics,
        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_swerve::setModuleStates,
        m_swerve);
  
      SwerveControllerCommand swerveControllerCommandLeft = 
      new SwerveControllerCommand(
        leftTraj,
        m_swerve::getPose,
        m_swerve.m_kinematics,
        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_swerve::setModuleStates,
        m_swerve);
        */
  }

}
