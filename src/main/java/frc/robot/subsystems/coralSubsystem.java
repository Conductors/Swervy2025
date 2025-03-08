package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class coralSubsystem extends SubsystemBase {
    private SparkMax elevatorMotorA;  
    private SparkMax elevatorMotorB;
    private SparkMax tiltMotor;
    private Servo gateMotor;// = new Servo(8);
    private int elevatorMotorAPort = 24;
    private int elevatorMotorBPort = 25;
    private int tiltMotorPort = 26;
    private int gateMotorPort = 8;    

    private DutyCycleEncoder m_tiltEncoder;
    private int tiltEncoderPort = 5;
    private double m_desiredTiltAngle = 0;
    private double tiltMotorMaxSPeed = 1; //speed to spin the tilt motor
    private double m_ActualTiltAngle = 0;

    private double m_desiredGatePos = 0;
    private final double gateMotorOpenPos = .5;
    private final double gateMotorClosePos = 0;


    private RelativeEncoder m_ElevatorEncA;
    private RelativeEncoder m_ElevatorEncB;
    private double c_ElevatorBOffset = 0; //+add (Subtract) this from the Elevator B desired goal
    private double m_DesiredHeight = 0;
    private double elevatorMaxMotorSpeed = 1.0; //speed to lift the motors
    private double m_ActualHeightA = 0;
    private double m_ActualHeightB = 0;



    private ProfiledPIDController m_elevatorPIDA;
    private ProfiledPIDController m_elevatorPIDB;
    private ProfiledPIDController m_tiltMotorPID;
    

    public coralSubsystem() {
        elevatorMotorA = new SparkMax(elevatorMotorAPort, SparkLowLevel.MotorType.kBrushless);
        elevatorMotorB = new SparkMax(elevatorMotorBPort, SparkLowLevel.MotorType.kBrushless);

        tiltMotor = new SparkMax(tiltMotorPort, SparkLowLevel.MotorType.kBrushless);
        gateMotor = new Servo(gateMotorPort);

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
      m_ActualHeightA   = m_ElevatorEncA.getPosition();
      m_ActualHeightB   = m_ElevatorEncB.getPosition();
      m_ActualTiltAngle = m_tiltEncoder.get();

     /* elevatorMotorA.set(MathUtil.clamp(m_elevatorPIDA.calculate(m_ActualHeightA, m_DesiredHeight),
                                        -elevatorMaxMotorSpeed,
                                        elevatorMaxMotorSpeed));    //need to check motor direction
      elevatorMotorB.set(MathUtil.clamp(m_elevatorPIDB.calculate(m_ActualHeightB, m_DesiredHeight+c_ElevatorBOffset),
                                        -elevatorMaxMotorSpeed,
                                        elevatorMaxMotorSpeed));
    */                                    
      gateMotor.set(m_desiredGatePos);
  /*  
      tiltMotor.set(MathUtil.clamp(m_tiltMotorPID.calculate(m_ActualTiltAngle, m_desiredTiltAngle),
                                        -tiltMotorMaxSPeed,
                                        tiltMotorMaxSPeed));
  */
        //Publish Stuff to Dashboard
        SmartDashboard.putNumber("ElHeightDes", m_DesiredHeight);
        SmartDashboard.putNumber("ElHeightAct_A", m_ActualHeightA);
        SmartDashboard.putNumber("ElHeightAct_B", m_ActualHeightB);
        SmartDashboard.putNumber("DesiredTiltAngle", m_desiredTiltAngle);
        SmartDashboard.putNumber("DesiredTiltAngle", m_desiredTiltAngle);
        SmartDashboard.putNumber("DesiredGatepPos", m_desiredGatePos);
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

    public double getActualTiltAngle() {
        return m_ActualTiltAngle;
      }
      
    public void closeGate() {
        m_desiredGatePos = gateMotorClosePos;
    }

    public void openGate() {
        m_desiredGatePos = gateMotorOpenPos;
    }

    public double getDesiredGatePos() {
        return m_desiredGatePos;
    }

    
}

