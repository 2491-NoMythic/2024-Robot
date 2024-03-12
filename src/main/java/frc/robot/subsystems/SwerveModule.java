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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.settings.Constants.DriveConstants;

public class SwerveModule {
  
  private String m_moduleName;
  private final TalonFX m_driveMotor;
  private final TalonFX m_steerMotor;
  private final CANcoder m_steerEncoder;
  private final Rotation2d m_steerEncoderOffset;

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
    m_moduleName = moduleName;
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

  public void logToNT() {
    String networkTableName = "Modules/" + m_moduleName;
    SmartDashboard.putNumber(networkTableName + "/Steer/angleCurrent", m_steerMotor.getPosition().getValue());
    SmartDashboard.putNumber(networkTableName + "/Steer/angleDesired", m_desiredSteerAngle);
    SmartDashboard.putNumber(networkTableName + "/Steer/tempMotor", m_steerMotor.getDeviceTemp().getValue());
    SmartDashboard.putNumber(networkTableName + "/Steer/tempProcessor", m_steerMotor.getProcessorTemp().getValue());
    SmartDashboard.putBoolean(networkTableName + "/Steer/isPro", m_steerMotor.getIsProLicensed().getValue());
    SmartDashboard.putStringArray(networkTableName + "/Steer/control", controlToString(m_steerMotor.getAppliedControl()));


    SmartDashboard.putNumber(networkTableName + "/CanCoder/position", m_steerEncoder.getPosition().getValue());
    SmartDashboard.putString(networkTableName + "/CanCoder/magStatus", m_steerEncoder.getMagnetHealth().getValue().toString());
    SmartDashboard.putBoolean(networkTableName + "/CanCoder/isPro", m_steerEncoder.getIsProLicensed().getValue());

    SmartDashboard.putNumber(networkTableName + "/Drive/velocityCurrent", m_driveMotor.getVelocity().getValue());
    SmartDashboard.putNumber(networkTableName + "/Drive/velocityDesired", m_desiredDriveSpeed);
    SmartDashboard.putNumber(networkTableName + "/Drive/tempMotor", m_driveMotor.getDeviceTemp().getValue());
    SmartDashboard.putNumber(networkTableName + "/Drive/tempProcessor", m_driveMotor.getProcessorTemp().getValue());
    SmartDashboard.putBoolean(networkTableName + "/Drive/isPro", m_driveMotor.getIsProLicensed().getValue());
    SmartDashboard.putStringArray(networkTableName + "/Drive/control", controlToString(m_driveMotor.getAppliedControl()));
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