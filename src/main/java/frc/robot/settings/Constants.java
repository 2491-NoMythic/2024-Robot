// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.settings;

import javax.sound.midi.Patch;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  private Constants () {}

  public static final int LED_COUNT = 52;

  public static final class DriveConstants {
    public static final double ALLOWED_ERROR = 1;
    public enum Positions{
      FL(0),
      FR(0.25),
      BL(0.5),
      BR(0.75);

      private double rotation;

      Positions(double value) {
        rotation = value;

      }
      public double getValue() {
        return rotation;
      }
    }
    public enum Offsets{
      AUGIE(0.153818),
      BENH(0.153564),
      EVELYN(-0.111084),
      OMARIAHN(0.266846),
      PHOEBE(-0.2458), //Moira FL
      ROYCE(-0.0031), // Moira FR
      ROWAN(0.3916), // Moira BL
      QUINN(0.3557), //Moira BR
      // PHOEBE(0.253174), //Moira FL
      // ROYCE(-0.254639), // Moira FR
      // ROWAN(-0.113525), // Moira BL
      // QUINN(0.108154), //Moira BR
      LIAM(0),
      LEVI(-0.38501),
      WILLA(-0.152832), //Moira inverted FL
      OPAL(0.211419-0.25), //Moira inverted FR
      CLOVER(0.239746), //Moira inverted BL
      NICHOLAS(-0.164551); //Moira inverted BR

      private double offset;
      Offsets(double value) {
        offset = value;
      }
      public Rotation2d getValue(Positions pos) {
        return Rotation2d.fromRotations(MathUtil.inputModulus(offset+pos.getValue(), -0.5, 0.5));
      }
    }
    private DriveConstants() {
    }
    public static final Pose2d DRIVE_ODOMETRY_ORIGIN = new Pose2d(5.0, 5.0, new Rotation2d());
    /**
     * The bumper-to-bumper width of the robot.
     */
    public static final double DRIVETRAIN_ROBOT_WIDTH_METERS = 0.83;
    /**
     * The left-to-right distance between the drivetrain wheels
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.52705;
    /**
     * The front-to-back distance between the drivetrain wheels.
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.52705;

    /**
     * The diameter of the module's wheel in meters.
     */
    public static final double DRIVETRAIN_WHEEL_DIAMETER = 0.10033;

    /**
     * The overall drive reduction of the module. Multiplying motor rotations by
     * this value should result in wheel rotations.
     * these numbers are just gear ratios that are used. Ask build team about these.
     */
    public static final double DRIVETRAIN_DRIVE_REDUCTION = (15.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);

    /**
     * Whether the drive motor should be counterclockwise or clockwise positive. 
     * If there is an odd number of gear reductions this is typically clockwise-positive.
     */
    public static final InvertedValue DRIVETRAIN_DRIVE_INVERTED = InvertedValue.CounterClockwise_Positive;

    /**
     * The overall steer reduction of the module. Multiplying motor rotations by
     * this value should result in wheel rotations.
     */
    public static final double DRIVETRAIN_STEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);

    /**
     * Whether the steer motor should be counterclockwise or clockwise positive. 
     * If there is an odd number of gear reductions this is typically clockwise-positive.
     */
    public static final InvertedValue DRIVETRAIN_STEER_INVERTED = InvertedValue.CounterClockwise_Positive;

    /**
     * How many meters the wheels travel per rotation. <p>
     * Multiply rotations by this to get meters.<p>
     * Divide meters by this to get rotations.
     */
    public static final double DRIVETRAIN_ROTATIONS_TO_METERS = (DRIVETRAIN_WHEEL_DIAMETER * Math.PI);

    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight line.
     */
    /*
     * FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
     * The formula for calculating the theoretical maximum velocity is:
     * <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
        DRIVETRAIN_DRIVE_REDUCTION * DRIVETRAIN_WHEEL_DIAMETER * Math.PI;
    /**
     * The drive motor sensor value at a 100% duty cycle output in a straight line.
     */
    public static final double MAX_VELOCITY_RPS_EMPIRICAL = 15.697;
    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
        Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        // Front left
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Front right
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back left
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back right
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

    public static final String DRIVETRAIN_SMARTDASHBOARD_TAB = "Drivetrain";
    public static final String CANIVORE_DRIVETRAIN = "Swerve";
    public static final int DRIVETRAIN_PIGEON_ID = 0;

    public static final int FL_DRIVE_MOTOR_ID = 1;
    public static final int FL_STEER_MOTOR_ID = 2;
    public static final int FL_STEER_ENCODER_ID = 1;
    public static final Rotation2d FL_STEER_OFFSET = Rotation2d.fromRotations(0.272217);

    public static final int FR_DRIVE_MOTOR_ID = 3;
    public static final int FR_STEER_MOTOR_ID = 4;
    public static final int FR_STEER_ENCODER_ID = 2;
    public static final Rotation2d FR_STEER_OFFSET = Rotation2d.fromRotations(0.41333);

    public static final int BL_DRIVE_MOTOR_ID = 5;
    public static final int BL_STEER_MOTOR_ID = 6;
    public static final int BL_STEER_ENCODER_ID = 3;
    public static final Rotation2d BL_STEER_OFFSET = Rotation2d.fromRotations(-0.11792);

    public static final int BR_DRIVE_MOTOR_ID = 7;
    public static final int BR_STEER_MOTOR_ID = 8;
    public static final int BR_STEER_ENCODER_ID = 4;
    public static final Rotation2d BR_STEER_OFFSET = Rotation2d.fromRotations(0.403809);

    // Drive Motor
    public static final double k_DRIVE_P = 0.03;
    public static final double k_DRIVE_I = 0;
    public static final double k_DRIVE_D = 0;
    public static final double k_DRIVE_FF_S = 0;
    public static final double k_DRIVE_FF_V = 0;
    public static final double DRIVE_DEADBAND_MPS = 0.01;
    public static final double DRIVE_MOTOR_RAMP = 0.1;
    // Steer Motor
    /**
     * The maximum velocity of the steer motor. <p> 
     * This is the limit of how fast the wheels can rotate in place.
     */
    public static final double MAX_STEER_VELOCITY_RADIANS_PER_SECOND = Math.PI; // 1/2 rotation per second.
    /**
     * The maximum acceleration of the steer motor. <p>
     * This is the limit of how fast the wheels can change rotation speed.
     */
    public static final double MAX_STEER_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 2 * Math.PI; 
    public static final double k_STEER_P = 8;
    public static final double k_STEER_I = 0;
    public static final double k_STEER_D = 0; 
    public static final double k_STEER_FF_S = 0.0;
    public static final double k_STEER_FF_V = 0.0;

    // Auto PID loops
    // twin pid controllers that control the x and y robot movements.
    public static final double k_XY_P = 5;//*2.5;
    public static final double k_XY_I = 0.0;
    public static final double k_XY_D = 0.0;

    public static final double k_THETA_P = 1;
    public static final double k_THETA_I = 0.0;
    public static final double k_THETA_D = 0.0;
    public static final double k_THETA_TOLORANCE_DEGREES = 2.0;
    public static final double k_THETA_TOLORANCE_DEG_PER_SEC = 10;

    public static final double k_BALANCE_P = 0.025;
    public static final double k_BALANCE_I = 0.0;
    public static final double k_BALANCE_D = 0.0;
    public static final double k_BALANCE_TOLORANCE_DEGREES = 10.0;
    public static final double k_BALANCE_TOLORANCE_DEG_PER_SEC = 1;

    public static final PathConstraints DEFAUL_PATH_CONSTRAINTS = new PathConstraints(2, 1.5, Math.toRadians(360), Math.toRadians(360));

    public static final double k_PICKUP_NOTE_ta_P = 1;
    public static final double k_PICKUP_NOTE_ta_I = 0;
    public static final double k_PICKUP_NOTE_ta_D = 0;

    public static final double k_PICKUP_NOTE_tx_P = 1;
    public static final double k_PICKUP_NOTE_tx_I = 0;
    public static final double k_PICKUP_NOTE_tx_D = 0;
  }
public static final class ShooterConstants{
  public static final int SHOOTER_R_MOTORID = 2491;
  public static final int SHOOTER_L_MOTORID = 2491;
  public static final int PITCH_MOTOR_ID = 2491;
  public static final double SHOOTER_MOTOR_POWER = 1;
  public static final double SHOOTING_SPEED_MPS = 10;
  public static final double RUNNING_VELOCITY_RPS = 2491;
  public static final double ALLOWED_ERROR = 1;
  public static final double ALLOWED_SPEED_ERROR = 1;

  public static final double AUTO_AIM_ROBOT_kP = 0.125;
  public static final double AUTO_AIM_ROBOT_kI = 0.00;
  public static final double AUTO_AIM_ROBOT_kD = 0.00;
  
  public static final double AUTO_AIM_SHOOTER_kP = 2491;
  public static final double ROBOT_ANGLE_TOLERANCE = 0.5;
  public static final double SHOOTER_ANGLE_TOLERANCE = 0.5;
  public static final double SHOOTER_HEIGHT = 0.1;
  public static final double ANGLE_TICKS_PER_DEGREE = 2491;
  public static final double DEGREES_PER_ROTATION = 2491;
  public static final double DISTANCE_MULTIPLIER = 0;
  public static final double OFFSET_MULTIPLIER = 1;

 

  //PID coefficients for shooter:
  public static final double kP = 0.1;
  public static final double kI = 1e-4;
  public static final double kD = 1;
  public static final double kIz = 0;
  public static final double kFF = 0;
  public static final double kMaxOutput = 1;
  public static final double kMinOutput = -1;

  //PID values for pitch motor (changes angle of shooter):
  public static final double pitchkP = 2491;
  public static final double pitchkI = 2491;
  public static final double pitchkD = 2491;
  public static final double pitchkIz = 2491;
  public static final double pitchFeedForward = 2491;
  public static final double pitchMaxOutput = 2491;
  public static final double pitchMinOutput = 2491;
  public static final double shooterup = 2491;

}
public static final class ClimberConstants{
  public static final int CLIMBER_MOTOR_RIGHT = 2941;
  public static final int CLIMBER_MOTOR_LEFT = 2491;
  public static final double CLIMBER_SPEED_DOWN = 0.75;
  public static final double CLIMBER_SPEED_UP = 0.75;

  //TODO: refine these values
  public static final double CLIMBER_VOLTAGE_ALIGN = 0.75;
  public static final double CLIMBER_VOLTAGE_PULL_DOWN = 3;
  public static final double CLIMBER_RPM = 0.1;
}
public static final class IndexerConstants{
  public static final int INDEXER_MOTOR = 2491;
  public static final double INDEXER_SPEED = 2491;
}
public static final class IntakeConstants{
  public static final int INTAKE_1_MOTOR = 2491;
  public static final int INTAKE_2_MOTOR = 2491;
  public static final int BRUSH_1_MOTOR = 2491;
  public static final int BRUSH_2_MOTOR = 2491;
  public static final int BRUSH_3_MOTOR = 2491;
  public static final double INTAKE_SPEED = 0.75;
}
public static final class CTREConfigs {
  public TalonFXConfiguration driveMotorConfig;
  public TalonFXConfiguration steerMotorConfig;
  public CANcoderConfiguration steerEncoderConfig;
  public Pigeon2Configuration pigeon2Config;

  public CTREConfigs() {
      driveMotorConfig = new TalonFXConfiguration();
      steerMotorConfig = new TalonFXConfiguration();
      steerEncoderConfig = new CANcoderConfiguration();
      pigeon2Config = new Pigeon2Configuration();

      // Steer motor.
      steerMotorConfig.Feedback.RotorToSensorRatio = 1/DriveConstants.DRIVETRAIN_STEER_REDUCTION;
      steerMotorConfig.MotorOutput.Inverted = DriveConstants.DRIVETRAIN_STEER_INVERTED;
      // steerMotorConfig.MotorOutput.DutyCycleNeutralDeadband = 0.05;
      steerMotorConfig.Slot0.kP = DriveConstants.k_STEER_P;
      steerMotorConfig.Slot0.kI = DriveConstants.k_STEER_I;
      steerMotorConfig.Slot0.kD = DriveConstants.k_STEER_D;
      steerMotorConfig.Slot0.kS = DriveConstants.k_STEER_FF_S;
      steerMotorConfig.Slot0.kV = DriveConstants.k_STEER_FF_V;
      steerMotorConfig.Voltage.PeakForwardVoltage = 12;
      steerMotorConfig.Voltage.PeakReverseVoltage = -12;
      steerMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

      steerMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;
      // Drive motor.
      driveMotorConfig.Feedback.SensorToMechanismRatio = 1/DriveConstants.DRIVETRAIN_DRIVE_REDUCTION;
      driveMotorConfig.MotorOutput.Inverted = DriveConstants.DRIVETRAIN_DRIVE_INVERTED;
      driveMotorConfig.MotorOutput.DutyCycleNeutralDeadband = 0.0;
      driveMotorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = DriveConstants.DRIVE_MOTOR_RAMP;
      driveMotorConfig.Slot0.kP = DriveConstants.k_DRIVE_P;
      driveMotorConfig.Slot0.kI = DriveConstants.k_DRIVE_I;
      driveMotorConfig.Slot0.kD = DriveConstants.k_DRIVE_D;
      driveMotorConfig.Slot0.kS = DriveConstants.k_DRIVE_FF_S;
      driveMotorConfig.Slot0.kV = DriveConstants.k_DRIVE_FF_V;
      driveMotorConfig.Voltage.PeakForwardVoltage = 12;
      driveMotorConfig.Voltage.PeakReverseVoltage = -12;
      driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

      //  Steer encoder.
      steerEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

      // Pigeon 2.
      pigeon2Config.MountPose.MountPosePitch = 0;
      pigeon2Config.MountPose.MountPoseRoll = 0;
      pigeon2Config.MountPose.MountPoseYaw = 0;
  }
}
  public final class PS4Driver{
    private PS4Driver() {
    }
    public static final int DRIVE_CONTROLLER_ID = 0;
    /**Left stick Y-axis. <p> Left = -1 || Right = 1*/
    public static final int X_AXIS = 0; 
    /**Left stick X-axis. <p> Forwards = -1 || Backwards = 1*/
    public static final int Y_AXIS = 1;
    /**Right stick Z-axis. <p> Left = -1 || Right = 1*/
    public static final int Z_AXIS = 2;
    /**Right stick Z-rotate. <p> Forwards = -1 || Backwards = 1*/
    public static final int Z_ROTATE = 5;
    /**Value used to differentiate between angle 0 and rest position.*/
    public static final double NO_INPUT = 404;
    public static final double DEADBAND_NORMAL = 0.08;
    public static final double DEADBAND_LARGE = 0.1;
}

  public final class PS4Operator{
    public static final int OPERATOR_CONTROLLER_ID = 1;

  }

  public final class Field{
    public static final double BLUE_SPEAKER_X = 0.23;
    public static final double RED_SPEAKER_X = 16.49;
    public static final double SPEAKER_Y = 5.612;//16.412;
    public static final double SPEAKER_Z = 2.045; //height of opening
    public static final double MAX_SHOOTING_DISTANCE = 2491;

    public static final double AMPLIFIER_ANGLE = 110;

  }

public final class Vision{
  public static final String APRILTAG_LIMELIGHT2_NAME = "limelight-april2";
  public static final String APRILTAG_LIMELIGHT3_NAME = "limelight-april3";
  public static final String OBJ_DETECITON_LIMELIGHT_NAME = "limelight-neural";


  public static final Translation2d fieldCorner = new Translation2d(16.54, 8.02);

  public static final double K_DETECTOR_TX_P = 0.1;
  public static final double K_DETECTOR_TX_I = 0;
  public static final double K_DETECTOR_TX_D = 0;
  
  public static final double K_DETECTOR_TY_P = 0.1;
  public static final double K_DETECTOR_TY_I = 0;
  public static final double K_DETECTOR_TY_D = 0;
}
public final class PathConstants{
  //Welcome, to  Pathconstantic Park
  //Here the fine beasts of the Pathplanner Period reside, after being brought back through DNA
  //Middle of the Left-chain
  public static final Pose2d CLIMB_LM_POSE = new Pose2d(4.33, 4.88, new Rotation2d(116.98));
  //Middle of the Mid-chain
  public static final Pose2d CLIMB_MM_POSE = new Pose2d(5.91, 4.12, new Rotation2d(-0.90));
  //Middle of the Right-chain
  public static final Pose2d CLIMB_RM_POSE = new Pose2d(4.52, 3.24, new Rotation2d(-120.96));

  //Left side of the Left-chain
  public static final Pose2d CLIMB_LL_POSE = new Pose2d(2.491, 2.491, new Rotation2d(59.86));
  //Left side of the Mid-chain
  public static final Pose2d CLIMB_ML_POSE = new Pose2d( 2.491, 2.491, new Rotation2d(-179.64));
  //Left side of the Right-chain
  public static final Pose2d CLIMB_RL_POSE = new Pose2d( 2.491, 2.491, new Rotation2d(-179.64));
  
 //Right side of the Left-chain
  public static final Pose2d CLIMB_LR_POSE = new Pose2d(2.491, 2.491, new Rotation2d(59.86));
  //Right side of the Mid-chain
  public static final Pose2d CLIMB_MR_POSE = new Pose2d( 2.491, 2.491, new Rotation2d(-179.64));
  //Right side of the Right-chain
  public static final Pose2d CLIMB_RR_POSE = new Pose2d( 2.491, 2.491, new Rotation2d(-179.64));

  

  public static final Pose2d CLIMB__POSE = new Pose2d(12.19, 3.24, new Rotation2d(-61.23));

  public static final Pose2d AMP_RED_POSE = new Pose2d(1.93, 0.5, new Rotation2d(-89.33));
  public static final Pose2d AMP_BLUE_POSE = new Pose2d(1.93, 8, new Rotation2d(-89.33));

}
}
