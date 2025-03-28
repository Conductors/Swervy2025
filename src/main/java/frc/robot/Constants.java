package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {
    //RoboRio Constants
    public static final double kDefaultPeriod = 0.02; //50Hz

    //Drivetrain (whole robot) constants
    public static final double kMaxRobotSpeed = 3; // meters per second
    public static final double kMaxRobotSpeedLowGear = 0.8;
    public static final double kMaxRobotAngularSpeed = 2*Math.PI; // 1/2 rotation per second = Pi
    public static final double kMaxRobotAngularSpeedLowGear = Math.PI/2; // 1/2 rotation per second = Pi
    public static final double kWheelRadius = 0.0508;   //4" mk4 wheels
    public static final double kGearRatio = 8.1;  //actual 8.1
    public static final int kEncoderResolution = 42;



    //Swerve Module Configs
    public static final double kRobotWidth  = 0.303;      //Distance in x-coor from center of robot to swerve module
    public static final double kRobotLength = 0.303;      //Distance in y-coor from center of robot to swerve module
    public static final double kTurnMotorMaxAngSpeed = 2*Math.PI*2;     //2*pi*2 = 2 rotations per second

    //Front Left Swerve Module
    public static final double kFL_DriveChannel = 11;
    public static final double kFL_TurnChannel  = 10;
    public static final double kFL_TurnEncoderChannel = 0;
    public static final double kFL_TurnEncoderOffset = 0.634;

    public static final double kFR_DriveChannel = 15;
    public static final double kFR_TurnChannel  = 14;
    public static final double kFR_TurnEncoderChannel = 2;
    public static final double kFR_TurnEncoderOffset = 0.834;

    public static final double kBL_DriveChannel = 13;
    public static final double kBL_TurnChannel  = 12;
    public static final double kBL_TurnEncoderChannel = 1;
    public static final double kBL_TurnEncoderOffset = 0.412;

    public static final double kBR_DriveChannel = 17;
    public static final double kBR_TurnChannel  = 16;
    public static final double kBR_TurnEncoderChannel = 3;
    public static final double kBR_TurnEncoderOffset = 0.336;

    public static final double KpCrane = 0.5;
    public static final double CraneMaxAccel = Math.PI;
    public static final double CraneMaxVelocity = Math.PI;
    public static final double WristMaxAccel = 20;
    public static final double WristMaxVelocity = 5;
    public static final double KpWrist = 0.5;

    public static enum Position {
        keStow, keProcessor, keReef1, keReef2, keReef3, keReef4, keGround, keCoralStation, kNumOfPosition       //must keep this order for setpoint arrays
    };

    public static enum tiltPosition {
        keStow, keLoad, keScore
    };

    public static final class csConstants {
        public static final double[] k_ElevatorHeight = {
            0,      //keStow   
            -10,         //keProcessor
            -5,      //keReef1
            -40,        //keReef2
            -119,       //keReef3
            -271.7,       //keReef4
            0,       //keGround
            -10,       //Coral Station
        };

        public static final double[] k_tiltAngleSetpoint = {
            0.5,      //keStow  
            0.55,         //keLoad
            0.30       //keScore
        };
    }

    public static final class aGConstants {
        public static final int k_CraneMotorPort = 4;
        public static final int k_WristMotorPort = 5;
        public static final int k_ClawMotorPortUpper = 7;
        public static final int k_ClawMotorPortLower = 6;
        public static final int k_CraneEncPort = 4;
        public static final int k_WristEncPort = 8;
        public static final double k_CraneEncOffset = 0;
        public static final double k_WristEncOffset = 0;

        public static final double[] k_WristAngleSetpoint = {
            2.75,      //keStow
            3.05,      //keProcessor
            2.13,      //keReef1
            2.43,      //keReef2
            1.9,        //keReef3
            1.6,        //keReef4
            2.75,     //keGround
            2.75      //keCoralStation
        };
        public static final double[] k_CraneAngleSetpoint = {
            2.84,      //keStow
            2.16,      //keProcessor
            1.65,      //keReef1
            1.25,      //keReef2
            0.57,      //keReef3
            0.67,     //keReef4
            2.84,      //keGround
            2.84      //keCoralStation
        };  

        public static final double k_clawInSpeed = .5;
        public static final double k_clawOutSpeed = .5;


    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 30;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1.3;
        public static final double kPYController = 1.3;
        public static final double kPThetaController = 1;
    
        public static final double kPP_PXController = 5;
        public static final double kPP_PYController = 1.3;
        public static final double kPP_PThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        public static final String[] kAutoProgram = {"None", "Auto 1", "Auto 2", "Auto 3", "BackUp", "ScoreOneCoral", "ScoreCoralTake1Algae", "diagScoreReef", "ScoreCoralClearAlgae"};
      }

      public static final class coralConstants {

        public static final double kP_elevatorA = 0.1;
        public static final double kP_elevatorB = 0.1;
        public static final double kP_tilt      = 2.5;
        
        public static final double elevatorMaxVelocity = 100;
        public static final double elevatorMaxAccel = 1000;

        public static final double tiltMaxVelocity = 1;
        public static final double tiltMaxAccel = 1;
      }

}
