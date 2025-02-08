package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class coralSubsystem extends SubsystemBase {
    private SparkMax elevatorMotorA;  
    private SparkMax elevatorMotorB;
    private SparkMax tiltMotor;
    private SparkMax gateMotor;
    private int elevatorMotorAPort = 0;
    private int elevatorMotorBPort = 0;
    private int tiltMotorPort = 0;
    private int gateMotorPort = 0;    

    private DutyCycleEncoder m_tiltEncoder;
    private int tiltEncoderPort = 0;
    private double m_desiredTiltAngle = 0;
    private double tiltMotorMaxSPeed = 1.; //speed to spin the tilt motor

    private final double gateMotorSpeed = .25; //speed to rotate the gateMotor
    private double m_desiredGatePos = 0;
    private final double gateMotorOpenPos = 1;
    private final double gateMotorClosePos = 2;

    private RelativeEncoder m_ElevatorEncA;
    private RelativeEncoder m_ElevatorEncB;
    private double c_ElevatorBOffset = 0; //+add (Subtract) this from the 
    private double m_DesiredHeight = 0;
    private double elevatorMaxMotorSpeed = 1.0; //speed to lift the motors
    private double m_ActualHeightA = 0;
    private double m_ActualHeightB = 0;


    private ProfiledPIDController m_elevatorPIDA;
    private ProfiledPIDController m_elevatorPIDB;
    private ProfiledPIDController m_tiltMotorPID;


    public coralSubsystem() {
        elevatorMotorA = new SparkMax(0, SparkLowLevel.MotorType.kBrushless);
        elevatorMotorB = new SparkMax(0, SparkLowLevel.MotorType.kBrushless);

        tiltMotor = new SparkMax(0, SparkLowLevel.MotorType.kBrushless);
        gateMotor = new SparkMax(0, SparkLowLevel.MotorType.kBrushless);

        m_tiltEncoder = new DutyCycleEncoder(tiltEncoderPort);
        m_ElevatorEncA = elevatorMotorA.getEncoder();
        m_ElevatorEncB = elevatorMotorB.getEncoder();

        m_elevatorPIDA =  new ProfiledPIDController(
          Constants.coralConstants.kP_elevatorA,
          0,
          0,
          new TrapezoidProfile.Constraints(
            Constants.coralConstants.elevatorMaxVelocity,
            Constants.coralConstants.elevatorMaxAccel));
        m_elevatorPIDA.setTolerance(.05);

        m_elevatorPIDB =  new ProfiledPIDController(
          Constants.coralConstants.kP_elevatorB,
          0,
          0,
          new TrapezoidProfile.Constraints(
              Constants.coralConstants.elevatorMaxVelocity,
              Constants.coralConstants.elevatorMaxAccel));
        m_elevatorPIDB.setTolerance(.05);
    
        m_tiltMotorPID =  new ProfiledPIDController(
          Constants.coralConstants.kP_tilt,
          0,
          0,
          new TrapezoidProfile.Constraints(
              Constants.coralConstants.tiltMaxVelocity,
              Constants.coralConstants.tiltMaxAccel));
              m_tiltMotorPID.setTolerance(.05);
    }

    @Override
    public void periodic() {
      m_ActualHeightA = m_ElevatorEncA.getPosition();
      m_ActualHeightB = m_ElevatorEncB.getPosition();
  
      
      elevatorMotorA.set(m_elevatorPIDA.calculate(m_ActualHeightA, m_DesiredHeight));    //need to check motor direction
      craneMotor.set(m_WristPIDController.calculate(actualWristAngle, desiredWristAngle));
  
      clawMotor.set(desiredClawSpeed);
  
    }

    public void setDesiredHeight(double height) {
        m_DesiredHeight = height;
    }

    public double getDesiredHeight() {
        return m_DesiredHeight;
    }

    /**
     *@param angle Angle in radians
     */
    public void setDesiredTiltAngle(double angle) {
        m_desiredTiltAngle = angle;
    }

    public double getDesiredTiltAngle() {
        return m_desiredTiltAngle;
    }

    public void closeGate() {
        m_desiredGatePos = gateMotorClosePos;
    }

    public void openGate() {
        m_desiredGatePos = gateMotorOpenPos;
    }

    public double getGatePos() {
        return m_desiredGatePos;
    }


}

