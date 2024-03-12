// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.settings.Constants.DriveConstants;

public class SwerveModule {

  private final TalonFX m_driveMotor;
  private final TalonFX m_steerMotor;
  private final CANcoder m_steerEncoder;
  private final Rotation2d m_steerEncoderOffset;

  private DoublePublisher log_SteerAngleCurrent;
  private DoublePublisher log_SteerAngleDesired;
  private StringArrayPublisher log_SteerControl;
  private BooleanPublisher log_SteerIsPro;
  private DoublePublisher log_SteerMotorTemp;
  private DoublePublisher log_SteerProcessorTemp;

  private DoublePublisher log_CanCoderPosition;
  private StringPublisher log_CanCoderMagStatus;
  private BooleanPublisher log_CanCoderIsPro;

  private DoublePublisher log_DriveVelocityCurrent;
  private DoublePublisher log_DriveVelocityDesired;
  private StringArrayPublisher log_DriveControl;
  private BooleanPublisher log_DriveIsPro;
  private DoublePublisher log_DriveMotorTemp;
  private DoublePublisher log_DriveProcessorTemp;

  ShuffleboardTab debugInfo;
  /**
   * The target wheel angle in rotations. [-.5, .5]
   */
  private double m_desiredSteerAngle;
  /**
   * The target wheel speed in rotations per second
   */
  private double m_desiredDriveSpeed;

  private VelocityDutyCycle m_driveControl = new VelocityDutyCycle(0);
  private PositionDutyCycle m_steerControl = new PositionDutyCycle(0);
  private NeutralOut m_neutralControl = new NeutralOut();

  /**
   * Constructs a SwerveModule with a drive motor, turning motor and turning encoder.
   *
   * @param driveMotorChannel
   * @param steerMotorChannel
   * @param steerEncoderChannel
   * @param steerEncoderOffset
   * @param canivoreName
   */
  public SwerveModule(
      String moduleName,
      int driveMotorChannel,
      int steerMotorChannel,
      int steerEncoderChannel,
      Rotation2d steerEncoderOffset,
      String canivoreName) {
    m_driveMotor = new TalonFX(driveMotorChannel, canivoreName);
    m_steerMotor = new TalonFX(steerMotorChannel, canivoreName);
    m_steerEncoder = new CANcoder(steerEncoderChannel, canivoreName);
    m_steerEncoderOffset = steerEncoderOffset;

    CANcoderConfiguration steerEncoderConfig = DrivetrainSubsystem.ctreConfig.steerEncoderConfig;
    TalonFXConfiguration steerMotorConfig = DrivetrainSubsystem.ctreConfig.steerMotorConfig;

    steerEncoderConfig.MagnetSensor.MagnetOffset = -m_steerEncoderOffset.getRotations();
    steerMotorConfig.Feedback.FeedbackRemoteSensorID = steerEncoderChannel;
    // Apply the configurations.
    m_driveMotor.getConfigurator().apply(DrivetrainSubsystem.ctreConfig.driveMotorConfig);
    m_steerMotor.getConfigurator().apply(steerMotorConfig);
    m_steerEncoder.getConfigurator().apply(steerEncoderConfig);
    
    logInit(moduleName);
  }

  /**
   * Returns the current state of the module.
      * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getSpeedMetersPerSecond(), getRotation());
  }
  /**
   * Returns the current position of the module.
      * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDriveDistanceMeters(), getRotation());
  }
  /**
   * Returns the current encoder angle of the steer motor.
      * @return The current encoder angle of the steer motor.
   */
  public Rotation2d getRotation() {
    return Rotation2d.fromRotations(MathUtil.inputModulus(m_steerMotor.getPosition().getValue(), -0.5, 0.5));
  }
  /**
   * Returns the target angle of the wheel.
      * @return The target angle of the wheel in degrees.
   */
  public double getTargetAngle() {
    return m_desiredSteerAngle;
  }
/**finds the curernt encoder position, it removes the current offset so we just get the raw position
      * @return
   */
  public double findOffset() {
    return MathUtil.inputModulus(
        (m_steerEncoder.getPosition().getValue()+m_steerEncoderOffset.getRotations()),
        -0.5,
        0.5);
  }
  /**
   * Returns the target speed of the wheel.
      * @return The target speed of the wheel in meters/second.
   */
  public double getTargetSpeedMetersPerSecond() {
    return m_desiredDriveSpeed;
  }
  /**
   * Returns the current encoder velocity of the drive motor.
      * @return The current velocity of the drive motor in meters/second.
   */
  public double getSpeedMetersPerSecond() {
    return (m_driveMotor.getVelocity().getValue() * DriveConstants.DRIVETRAIN_ROTATIONS_TO_METERS);
  }
  /**
   * Returns the current encoder distance of the drive motor.
      * @return The current distance of the drive motor in meters.
   */
  public double getDriveDistanceMeters() {
    return (m_driveMotor.getPosition().getValue() * DriveConstants.DRIVETRAIN_ROTATIONS_TO_METERS);
  }
/**Don't use */
  public void stop() {
    m_driveMotor.setControl(m_neutralControl);
    m_steerMotor.setControl(m_neutralControl);
  }
  /**
   * Sets the desired state for the module.
      * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    if (desiredState.angle == null) {
      DriverStation.reportWarning("Cannot set module angle to null.", true);
    }

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState state =
    SwerveModuleState.optimize(desiredState, getRotation());

    m_desiredSteerAngle = MathUtil.inputModulus(state.angle.getRotations(), -0.5, 0.5);
    m_desiredDriveSpeed = state.speedMetersPerSecond / DriveConstants.DRIVETRAIN_ROTATIONS_TO_METERS;

    if (Math.abs(m_desiredDriveSpeed) <= 0.001) {
      m_driveMotor.setControl(m_neutralControl);
      m_steerMotor.setControl(m_steerControl.withPosition(m_desiredSteerAngle));
    } else {
      m_driveMotor.setControl(m_driveControl.withVelocity(m_desiredDriveSpeed).withFeedForward(m_desiredDriveSpeed/DriveConstants.MAX_VELOCITY_RPS_EMPIRICAL));//TODO verify that this feedforward is effective
      m_steerMotor.setControl(m_steerControl.withPosition(m_desiredSteerAngle));
    }
  }

  public void logInit(String moduleName) {
    String networkTableName = "Modules/" + moduleName;
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    log_SteerAngleCurrent = inst.getDoubleTopic(networkTableName + "/Steer/angleCurrent").publish(/*PubSubOption.sendAll(true)*/);
    log_SteerAngleDesired = inst.getDoubleTopic(networkTableName + "/Steer/angleDesired").publish();
    log_SteerMotorTemp = inst.getDoubleTopic(networkTableName + "/Steer/tempMotor").publish();
    log_SteerProcessorTemp = inst.getDoubleTopic(networkTableName + "/Steer/tempProcessor").publish();
    log_SteerIsPro = inst.getBooleanTopic(networkTableName + "/Steer/isPro").publish();
    log_SteerControl = inst.getStringArrayTopic(networkTableName + "/Steer/control").publish();

    log_CanCoderPosition = inst.getDoubleTopic(networkTableName + "/CanCoder/position").publish();
    log_CanCoderMagStatus = inst.getStringTopic(networkTableName + "/CanCoder/magStatus").publish();
    log_CanCoderIsPro = inst.getBooleanTopic(networkTableName + "/CanCoder/isPro").publish();

    log_DriveVelocityCurrent = inst.getDoubleTopic(networkTableName + "/Drive/velocityCurrent").publish();
    log_DriveVelocityDesired = inst.getDoubleTopic(networkTableName + "/Drive/velocityDesired").publish();
    log_DriveMotorTemp = inst.getDoubleTopic(networkTableName + "/Drive/tempMotor").publish();
    log_DriveProcessorTemp = inst.getDoubleTopic(networkTableName + "/Drive/tempProcessor").publish();
    log_DriveIsPro = inst.getBooleanTopic(networkTableName + "/Drive/isPro").publish();
    log_DriveControl = inst.getStringArrayTopic(networkTableName + "/Drive/control").publish();
  }

  public void logToNT() {
    log_SteerAngleCurrent.set(m_steerMotor.getPosition().getValue());
    log_SteerAngleDesired.set(m_desiredSteerAngle);
    log_SteerMotorTemp.set(m_steerMotor.getDeviceTemp().getValue());
    log_SteerProcessorTemp.set(m_steerMotor.getProcessorTemp().getValue());
    log_SteerIsPro.set(m_steerMotor.getIsProLicensed().getValue());
    log_SteerControl.set(controlToString(m_steerMotor.getAppliedControl()));

    log_CanCoderPosition.set(m_steerEncoder.getPosition().getValue());
    log_CanCoderMagStatus.set(m_steerEncoder.getMagnetHealth().getValue().toString());
    log_CanCoderIsPro.set(m_steerEncoder.getIsProLicensed().getValue());

    log_DriveVelocityCurrent.set(m_driveMotor.getVelocity().getValue());
    log_DriveVelocityDesired.set(m_desiredDriveSpeed);
    log_DriveMotorTemp.set(m_driveMotor.getDeviceTemp().getValue());
    log_DriveProcessorTemp.set(m_driveMotor.getProcessorTemp().getValue());
    log_DriveIsPro.set(m_driveMotor.getIsProLicensed().getValue());
    log_DriveControl.set(controlToString(m_driveMotor.getAppliedControl()));

  }

  private String[] controlToString(ControlRequest control) {
    Map<String, String> mappedData = control.getControlInfo();
    List<String> output = new ArrayList<String>();
    mappedData.forEach(
        (key, value) -> {
          output.add(key + ":" + value);
        });
    return output.toArray(new String[output.size()]);
  }
}