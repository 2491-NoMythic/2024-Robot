// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  TalonFX shooterR;
  TalonFX shooterL;

  int runsValid;

  /** Creates a new Shooter. */
  public ShooterSubsystem() {
    Slot0Configs leftPIDConfigs;
    Slot0Configs rightPIDConfigs;

    if (Preferences.getBoolean("CompBot", true)) {
      rightPIDConfigs = new Slot0Configs().withKP(ShooterConstants.CompRightkP).withKV(ShooterConstants.CompRightkFF);
      leftPIDConfigs = new Slot0Configs().withKP(ShooterConstants.CompLeftkP).withKV(ShooterConstants.CompLeftkFF);
    } else {
      rightPIDConfigs = new Slot0Configs().withKP(ShooterConstants.PrackP).withKV(ShooterConstants.PrackFF);
      leftPIDConfigs = new Slot0Configs().withKP(ShooterConstants.PrackP).withKV(ShooterConstants.PrackFF);
    }

    runsValid = 0;
    shooterR = new TalonFX(ShooterConstants.SHOOTER_R_MOTORID);
    shooterL = new TalonFX(ShooterConstants.SHOOTER_L_MOTORID);
    shooterL.getConfigurator().apply(new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Coast))
        .withSlot0(leftPIDConfigs)
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(ShooterConstants.CURRENT_LIMIT).withStatorCurrentLimitEnable(true)));
    shooterL.getConfigurator().apply(new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive).withNeutralMode(NeutralModeValue.Coast))
        .withSlot0(rightPIDConfigs)
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(ShooterConstants.CURRENT_LIMIT).withStatorCurrentLimitEnable(true))
        .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(0.5)));

    shooterL.setControl(shooterL.getAppliedControl());
    shooterL.setControl(shooterR.getAppliedControl());
  }

  public void shoot(double runSpeed) {
    shoot(runSpeed, runSpeed);
  }

  public void shoot(double runSpeedLeft, double runSpeedRight) {
    shooterL.setControl(new DutyCycleOut(runSpeedLeft));
    shooterR.setControl(new DutyCycleOut(runSpeedRight));
  }

  public void shootRPS(double rps) {
    shootRPS(rps, rps);
  }

  public void shootRPS(double rpsLeft, double rpsRight) {
    shooterL.setControl(new VelocityDutyCycle(rpsLeft).withSlot(0));
    shooterR.setControl(new VelocityDutyCycle(rpsRight).withSlot(0));
  }

  private double getError() {
    return Math.abs(shooterR.getClosedLoopError().getValueAsDouble());
  }

  public boolean validShot() {
    return runsValid >= Constants.LOOPS_VALID_FOR_SHOT;
  }

  public void stop() {
    shoot(0);
  }

  @Override
  public void periodic() {
    if (DriverStation.isTest()) {
      SmartDashboard.putNumber("TESTING shooter speed error", getError());
      SmartDashboard.putNumber("shooter current right", shooterR.getSupplyCurrent().getValueAsDouble());
      SmartDashboard.putNumber("shooter current left", shooterL.getSupplyCurrent().getValueAsDouble());
    }
    if (getError() < ShooterConstants.ALLOWED_SPEED_ERROR) {
      runsValid++;
    } else {
      runsValid = 0;
    }
  }
}
