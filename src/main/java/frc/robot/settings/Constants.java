// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.settings;

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

  public static final int LOOPS_VALID_FOR_SHOT = 20;
  
  public static final class DriveConstants {
    public static final double ALLOWED_ERROR = 2;
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
    public static final double DRIVETRAIN_WHEEL_DIAMETER = 0.092;//0.098;

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
    public static final InvertedValue DRIVETRAIN_DRIVE_INVERTED = InvertedValue.Clockwise_Positive;

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
    public static final double DRIVE_CURRENT_LIMIT = 30;

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

    public static final double k_THETA_P = 4;
    public static final double k_THETA_I = 5.0;
    public static final double k_THETA_D = 0.0;
    public static final double k_THETA_TOLORANCE_DEGREES = 2.0;
    public static final double k_THETA_TOLORANCE_DEG_PER_SEC = 10;

    public static final double k_BALANCE_P = 0.025;
    public static final double k_BALANCE_I = 0.0;
    public static final double k_BALANCE_D = 0.0;
    public static final double k_BALANCE_TOLORANCE_DEGREES = 10.0;
    public static final double k_BALANCE_TOLORANCE_DEG_PER_SEC = 1;

    public static final PathConstraints DEFAULT_PATH_CONSTRAINTS = new PathConstraints(2, 1.5, Math.toRadians(360), Math.toRadians(360));

    public static final double k_PICKUP_NOTE_ta_P = 1;
    public static final double k_PICKUP_NOTE_ta_I = 0;
    public static final double k_PICKUP_NOTE_ta_D = 0;

    public static final double k_PICKUP_NOTE_tx_P = 1;
    public static final double k_PICKUP_NOTE_tx_I = 0;
    public static final double k_PICKUP_NOTE_tx_D = 0;
  }
public static final class ShooterConstants{
  public static final int SHOOTER_R_MOTORID = 10;
  public static final int SHOOTER_L_MOTORID = 9;
  public static final int PITCH_MOTOR_ID = 24;
  public static final double SHOOTER_MOTOR_POWER = 1;
  public static final double SHOOTER_AMP_POWER = 0.3;
  //power at 0.3 for bounce techinque, didnt work
  public static final double SHOOTING_SPEED_MPS = 19.665; //calculated with slo mo videos
  public static final double RUNNING_VELOCITY_RPS = 2491;
  public static final double ALLOWED_ANGLE_ERROR = 1.5;
  public static final double ALLOWED_SPEED_ERROR = 2;

  public static final double CURRENT_LIMIT = 100; //amps the motor is limited to

  public static final double AUTO_AIM_ROBOT_kP = 0.125;
  public static final double AUTO_AIM_ROBOT_kI = 0.00;
  public static final double AUTO_AIM_ROBOT_kD = 0.00;
  
  public static final double LONG_SHOOTING_RPS = 120;
  public static final double SHORT_SHOOTING_RPS = 100;
  public static final double PRAC_AMP_RPS = 8.0;
  public static final double COMP_AMP_RPS = 6.1;
  public static final double SUBWOOFER_RPS = SHORT_SHOOTING_RPS;
  public static final double PASS_RPS = 80;

  //the PID values used on the PID loop on the motor controller that control the position of the shooter angle
  public static final double ANGLE_SHOOTER_POWER_KP = 0.026;
  public static final double ANGLE_SHOOTER_POWER_KI = 0;
  public static final double ANGLE_SHOOTER_POWER_KD = 0.001;
  public static final double pitchFeedForward = 0.001;
  public static final double pitchMaxOutput = 1;
  public static final double pitchMinOutput = -1;
  
  public static final double AUTO_AIM_SHOOTER_kP = 0.003;
  public static final double ROBOT_ANGLE_TOLERANCE = 0.5;
  public static final double SHOOTER_ANGLE_TOLERANCE = 0.5;
  public static final double SHOOTER_HEIGHT = 0.254;
  public static final double ANGLE_TICKS_PER_DEGREE = 2491;
  public static final double DEGREES_PER_ROTATION = 360;
  public static final double DISTANCE_MULTIPLIER = 0.15;
  public static final double OFFSET_MULTIPLIER = 1.5;
  public static final double MINIMUM_SHOOTER_ANGLE = 11.64;
  public static final double COMP_MAXIMUM_SHOOTER_ANGLE = 108;
  public static final double PRAC_MAXIMUM_SHOOTER_ANGLE = 101;
  public static final double HUMAN_PLAYER_ANGLE = 97;
  public static final double HUMAN_PLAYER_RPS = -15;
  public static final double SAFE_SHOOTER_ANGLE = 15;
  public static final double GROUND_INTAKE_SHOOTER_ANGLE = 69;

  public static final double OVER_STAGE_PASS_ANGLE = 45;
  /**
   * the values used when adjusting the shooter's angle based on our speaker distance. Here's how we calculated them:
   * <p>
   * shoot at the speaker from various distance, and calculate how far under the target we miss (could be negative if we miss above the target). Then graph these points on desmos where the 
   * x-axis is distance-from-speaker (as calculated by the code), and the y-axis is how far under the target we missed. Using desmos, find the equation that fits these points as best we can. This might
   * be a quadratic, cubic, or maybe exponential equation. Then, use the a, b, and c values from desmos as these constants.
   */
  public static final class AdjustEquation {
  public static final double PRAC_ADJUST_EQUATION_A = 1.14168;
  public static final double PRAC_ADJUST_EQUATION_B = -1.22979;
  public static final double COMP_ADJUST_EQUATION_A = -0.00505024;
  public static final double COMP_ADJUST_EQUATION_B = 0.0651897;
  public static final double COMP_ADJUST_EQUATION_C = -0.0811224;
  public static final double COMP_ADJUST_EQUATION_D = 0.0456604;

  }
  // public static final double COMP_ADJUST_EQUATION_D = 1; unused becuase we aren't using a cubic equation
/**
 * the value that the encoder thinks is zero. when the shooter is resting on the bottom stop, the encoder should read 11.63 
 */
  public static final double CompBotZeroOffset = 155.2;//before rebuild was 334.7. After rebuild right away was 155.57
  public static final double PracBotZeroOffset = 83;

 

  //PID coefficients for shooter
  public static final double PracLeftkP = 0.01;
  public static final double PracRightkP = 0.01;
  public static final double PracLeftkFF  = 0.0105;
  public static final double PracRightkFF = 0.0068;
  // public static final double kI = 0;
  // public static final double kD = 0;
  public static final double kIz = 0;
  public static final double CompRightkFF = 0.0053;
  public static final double CompLeftkFF = 0.0098;
  public static final double CompRightkP = 0.012;
  public static final double CompLeftkP = 0.012;
  public static final double kMaxOutput = 1;
  public static final double kMinOutput = -1;

  //PID values for pitch motor (changes angle of shooter):
  public static final double shooterup = 45;

}
public static final class ClimberConstants{
  public static final int CLIMBER_MOTOR_RIGHT = 23;
  public static final int CLIMBER_MOTOR_LEFT = 22;
  public static final double CLIMBER_SPEED_DOWN = 1;
  public static final double CLIMBER_SPEED_UP = -1;
  public static final double MAX_MOTOR_ROTATIONS = 235;

  //TODO: refine these values
  public static final double CLIMBER_VOLTAGE_ALIGN = 0.25;
  public static final double CLIMBER_VOLTAGE_PULL_DOWN = 3;
  public static final double CLIMBER_RPM = 0.1;
  public static final int Right_HALL_EFFECT_INPUT = 9;
  public static final int LEFT_HALL_EFFECT_INPUT = 8;
}
public static final class IndexerConstants{
  public static final int INDEXER_MOTOR = 11;
  public static final int CURRENT_LIMIT = 50;
  public static final double INDEXER_INTAKE_SPEED = 1*0.6;//0.903 ;//speed to pick up at 10 ft/s
  public static final double HUMAN_PLAYER_INDEXER_SPEED = -0.5;//should be 0.5 TODO change to positive
  public static final double INDEXER_SHOOTING_RPS = 90;
  public static final double INDEXER_SHOOTING_POWER = 1*0.6;
  public static final double PRAC_INDEXER_AMP_SPEED = 170;
  public static final double COMP_INDEXER_AMP_SPEED = 135;
  public static final double INDEXER_KS = 0.35;
  public static final double INDEXER_KV = 0.103;
  public static final double INDEXER_KA = 0.01;
  public static final double INDEXER_KP = 0.15;
  /** the velocity to target when moving forward a set distance. in RPS */
  public static final double INDEXER_CRUISE_VELOCITY = 40;
  /** the acceleration to target target when moving to a set speed. In RPS^2*/
  public static final double INDEXER_ACCELERATION = 800; // at 400 acceleration, the motor takes 0.1 seconds to reach 40 RPS
  /** the jerk to target when moving to a set acceleration. In RPS^3.  */
  public static final double INDEXER_JERK = 8000; // at 4000 jerk, the motor takes 0.4 seconds to reach 400 RPS^2
  /**
   * multiply motor rotations by this value to get inches. Divide inches by this value to get motor rotations.
   */
  public static final double MOTOR_ROTATIONS_TO_INCHES = 0.71;

  public static final double AMP_SHOT_INCHES = 16;
}
public static final class IntakeConstants{
  public static final int INTAKE_1_MOTOR = 20;
  public static final int INTAKE_2_MOTOR = 21;
public static final int INTAKE_SIDE_MOTOR_LEFT = 25;
    public static final int INTAKE_SIDE_MOTOR_RIGHT = 26;
  public static final int BRUSH_1_MOTOR = 2491;
  public static final int BRUSH_2_MOTOR = 2491;
  public static final int BRUSH_3_MOTOR = 2491;
    /** the desired speed for the intake when doing ground intake. In Percent-of-full-power, from -1 to 1 */
  public static final double INTAKE_SPEED = 0.65*0.6;//0.5834 ; // ground intakes speed. 0.4167 picks up at 10 ft/s
  public static final double INTAKE_SIDE_SPEED = 0.65*0.6;
  public static final double INTAKE_MAX_VELOCITY = 11000;

  public static final double INTAKE_1_kP = 0.00002;
  public static final double INTAKE_1_kI = 0;
  public static final double INTAKE_1_kD = 0.0008;
  public static final double INTAKE_1_kFF = 0.000085;
  public static final double INTAKE_2_kP = 0.00007;
  public static final double INTAKE_2_kI = 0;//0.7;
  public static final double INTAKE_2_kD = 0;//0.002;
  public static final double INTAKE_2_kF = 0.00009;
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
      driveMotorConfig.Slot0.kP = DriveConstants.k_DRIVE_P*12;
      driveMotorConfig.Slot0.kI = DriveConstants.k_DRIVE_I*12;
      driveMotorConfig.Slot0.kD = DriveConstants.k_DRIVE_D*12;
      driveMotorConfig.Slot0.kS = DriveConstants.k_DRIVE_FF_S;
      driveMotorConfig.Slot0.kV = DriveConstants.k_DRIVE_FF_V;
      driveMotorConfig.Voltage.PeakForwardVoltage = 12;
      driveMotorConfig.Voltage.PeakReverseVoltage = -12;
      driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      driveMotorConfig.CurrentLimits.SupplyCurrentLimit = DriveConstants.DRIVE_CURRENT_LIMIT;
      driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      driveMotorConfig.CurrentLimits.StatorCurrentLimitEnable = false;
      driveMotorConfig.CurrentLimits.SupplyCurrentThreshold = 50;
      driveMotorConfig.CurrentLimits.SupplyTimeThreshold = 0.8;

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
    public static final int OPERATOR_CONTROLLER_ID = 1;
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

  }

  public final class Field{
    
    public static final double ROBOT_CENTER_TO_BUMBER = 0.419;
    public static final double AMP_SIDE_OUTER_TAPE_CORNER_BLUE_ROBOT_X = 2.945;//NOT fOUND
    public static final double AMP_SIDE_OUTER_TAPE_CORNER_BLUE_ROBOT_Y = 4.585; //NOT FOUND
    public static final double AMP_SIDE_OUTER_TAPE_CORNER_RED_ROBOT_X = 13.94;
    public static final double AMP_SIDE_OUTER_TAPE_CORNER_RED_ROBOT_Y = 3.99;
    public static final double AMP_SIDE_OUTER_TAPE_CORNER_RED_X = AMP_SIDE_OUTER_TAPE_CORNER_RED_ROBOT_X-ROBOT_CENTER_TO_BUMBER;
    public static final double AMP_SIDE_OUTER_TAPE_CORNER_RED_Y = AMP_SIDE_OUTER_TAPE_CORNER_RED_ROBOT_Y+ROBOT_CENTER_TO_BUMBER;
    public static final double AMP_SIDE_OUTER_TAPE_CORNER_BLUE_X = AMP_SIDE_OUTER_TAPE_CORNER_BLUE_ROBOT_X+ROBOT_CENTER_TO_BUMBER;
    public static final double AMP_SIDE_OUTER_TAPE_CORNER_BLUE_Y = AMP_SIDE_OUTER_TAPE_CORNER_BLUE_ROBOT_Y-ROBOT_CENTER_TO_BUMBER;

    public static final double CALCULATED_SHOOTER_RED_SPEAKER_X = AMP_SIDE_OUTER_TAPE_CORNER_RED_X+3.213;
    public static final double CALCULATED_RED_SPEAKER_Y = AMP_SIDE_OUTER_TAPE_CORNER_RED_Y+1.263;
    public static final double RED_SPEAKER_Y = 5.58;//home field: 5.613 HCPA: 5.68
    public static final double SHOOTER_RED_SPEAKER_X = 16.5;//home field: 16.582 HCPA: 16.55
    public static final double ROBOT_RED_SPEAKER_X = SHOOTER_RED_SPEAKER_X-0.165;//changed so that shots from the side wil aim to the opposite side, and bank in
    
    public static final double CALCULATED_SHOOTER_BLUE_SPEAKER_X = AMP_SIDE_OUTER_TAPE_CORNER_BLUE_X-3.213; //changed so that shots from the side wil aim to the opposite side, and bank in
    public static final double CALCULATED_BLUE_SPEAKER_Y = AMP_SIDE_OUTER_TAPE_CORNER_BLUE_Y+1.263;
    public static final double BLUE_SPEAKER_Y = 5.372;//home field: HCPA: 5.348
    public static final double SHOOTER_BLUE_SPEAKER_X = -0.03; //HCPA: 0.13
    public static final double ROBOT_BLUE_SPEAKER_X = SHOOTER_BLUE_SPEAKER_X+0.2;

    public static final double SPEAKER_Z = 2.08;//1.5;//1.8;//2.08; //height of opening. Changed so that the smaller spekeaker_x shots will still go in
    
    public static final double MAX_SHOOTING_DISTANCE = 9;
    public static final double SHORT_RANGE_SHOOTING_DIST = 3;

    public static final double AMPLIFIER_SHOOTER_ANGLE = 108;
    public static final double SUBWOOFER_ANGLE = 60;
    public static final double PODIUM_SHOOTER_ANGLE = 40;//36.3;
    public static final double FAR_STAGE_SHOOTER_ANGLE = 25.5;
    //24.5;
    public static final double OPPOSITE_STAGE_SHOOTER_ANGLE = 26;//24.5;
    
    public static final double BLUE_PODIUM_ROBOT_ANGLE = 149;
    public static final double RED_PODIUM_ROBOT_ANGLE = 31;
    public static final double BLUE_FAR_STAGE_ROBOT_ANGLE = 184;
    public static final double RED_FAR_STAGE_ROBOT_ANGLE = -4;
    public static final double RED_OPPOSITE_STAGE_ROBOT_ANGLE = 32;//31
    public static final double BLUE_OPPOSITE_STAGE_ROBOT_ANGLE = 148;
    public static final double RED_OVER_STAGE_PASS_ANGLE = 45;
    public static final double BLUE_OVER_STAGE_PASS_ANGLE = 135;

    //angle at 60 for bounce techinque, didn't work
  }

public final class Vision{
  public static final String APRILTAG_LIMELIGHT2_NAME = "limelight-aprill";
  public static final String APRILTAG_LIMELIGHT3_NAME = "limelight-aprilr";
  public static final String OBJ_DETECTION_LIMELIGHT_NAME = "limelight-neural";

  public static final String LIMELIGHT_SHUFFLEBOARD_TAB = "Vision";
  
  public static final double ALLOWABLE_POSE_DIFFERENCE = 0.5;
  public static final double MAX_TAG_DISTANCE = 3;

  public static final Translation2d FIELD_CORNER = new Translation2d(16.54, 8.02);

  public static final double K_DETECTOR_TX_P = 0.1;
  public static final double K_DETECTOR_TX_I = 0;
  public static final double K_DETECTOR_TX_D = 0;
  
  public static final double K_DETECTOR_TY_P = 0.1;
  public static final double K_DETECTOR_TY_I = 0;
  public static final double K_DETECTOR_TY_D = 0;

  // how many degrees back is your limelight rotated from perfectly vertical?
  public static final double limelightMountAngleDegrees = 22.0;
  // distance from the center of the Limelight lens to the floor
  public static final double limelightLensHeightInches = 0.233; 
  //height of april tags from the floor
  public static final double AprilTagHeight = 1.335;
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
