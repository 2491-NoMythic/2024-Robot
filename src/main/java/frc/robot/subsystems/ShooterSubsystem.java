 // Copyright (c) FIRST and other WPILib contributors.
 // Open Source Software; you can modify and/or share it under the terms of
 // the WPILib BSD license file in the root directory of this project.
 
 package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
 import frc.robot.commands.AngleShooter;
import frc.robot.commands.RotateRobot;
import frc.robot.settings.Constants;
import  frc.robot.settings.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 
 public class ShooterSubsystem extends SubsystemBase {
   TalonFX shooterR;
   TalonFX shooterL;
   double runSpeed;
   TalonFXConfigurator configuratorR;
   TalonFXConfigurator configuratorL;

  double differenceAngle;
	 double currentHeading;
	 double m_DesiredShooterAngle;
 
  CurrentLimitsConfigs currentLimitConfigs;
   Slot0Configs PIDLeftconfigs; 
   Slot0Configs PIDRightconfigs; 
   RelativeEncoder encoder1;

   //speaker angle calculating variables:

	double turningSpeed;
	RotateRobot rotateRobot;
	AngleShooter angleShooter;
	int accumulativeTurns;
  int runsValid;
  
  /** Creates a new Shooter. */
  public ShooterSubsystem(double runSpeed) {
    if(Preferences.getBoolean("CompBot", true)) {
      PIDRightconfigs = new Slot0Configs().withKP(ShooterConstants.CompRightkP).withKV(ShooterConstants.CompRightkFF);
      PIDLeftconfigs = new Slot0Configs().withKP(ShooterConstants.CompLeftkP).withKV(ShooterConstants.CompLeftkFF);
  } else {
      PIDRightconfigs = new Slot0Configs().withKP(ShooterConstants.PrackP).withKV(ShooterConstants.PrackFF);
      PIDLeftconfigs = new Slot0Configs().withKP(ShooterConstants.PrackP).withKV(ShooterConstants.PrackFF);
    }
    runsValid = 0;
    shooterR = new TalonFX(ShooterConstants.SHOOTER_R_MOTORID);
    shooterL = new TalonFX(ShooterConstants.SHOOTER_L_MOTORID);
    shooterL.getConfigurator().apply(new TalonFXConfiguration());
    shooterR.getConfigurator().apply(new TalonFXConfiguration());
    shooterL.setInverted(true);
    shooterR.setInverted(false);
    shooterL.setControl(shooterR.getAppliedControl());
    shooterL.setNeutralMode(NeutralModeValue.Coast);
    shooterR.setNeutralMode(NeutralModeValue.Coast);
    
    configuratorR = shooterR.getConfigurator();
    configuratorL = shooterL.getConfigurator();
    configuratorL.apply(new FeedbackConfigs().withSensorToMechanismRatio(1).withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor).withRotorToSensorRatio(1));
    configuratorR.apply(new FeedbackConfigs().withSensorToMechanismRatio(0.5).withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor).withRotorToSensorRatio(1));
    
    currentLimitConfigs = new CurrentLimitsConfigs();
    currentLimitConfigs.SupplyCurrentLimit = ShooterConstants.CURRENT_LIMIT;
    currentLimitConfigs.SupplyCurrentLimitEnable = true;
    currentLimitConfigs.StatorCurrentLimit = 100;
    currentLimitConfigs.StatorCurrentLimitEnable = true;
    configuratorL.apply(currentLimitConfigs);
    configuratorR.apply(currentLimitConfigs);
    
     configuratorR.apply(PIDRightconfigs);
     configuratorL.apply(PIDLeftconfigs);
    }
   
    
    public void shootThing(double runSpeed) {
      shooterR.set(runSpeed);
      shooterL.set(runSpeed);
    }
    public void shootRPS(double RPS) {
      shooterR.setControl(new VelocityDutyCycle(RPS).withSlot(0));
      shooterL.setControl(new VelocityDutyCycle(RPS/2).withSlot(0));
    }
    public void shootSameRPS(double RPS) {
      shooterR.setControl(new VelocityDutyCycle(RPS).withSlot(0));
      shooterL.setControl(new VelocityDutyCycle(RPS).withSlot(0));
    }
    private double getError() {
      return Math.abs(shooterR.getClosedLoopError().getValueAsDouble());
    }
    public boolean isReving() {
      return shooterR.getVelocity().getValueAsDouble()>15;
    }
    public boolean validShot() {
      return runsValid >= Constants.LOOPS_VALID_FOR_SHOT;
    }
  public void turnOff(){
    shooterR.setControl(new DutyCycleOut(0));
    shooterL.setControl(new DutyCycleOut(0));
  }
  // public double getSpeedRPS() {
  //   return shooterR.getVelocity().asSupplier().get();
  // }
@Override
  public void periodic() {
    SmartDashboard.putNumber("TESTING shooter speed error", getError());
    SmartDashboard.getBoolean("shooter speed rev'ed", validShot());
    SmartDashboard.putNumber("shooter current right", shooterR.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("shooter current left", shooterL.getSupplyCurrent().getValueAsDouble());
    if(getError()<ShooterConstants.ALLOWED_SPEED_ERROR) {
      runsValid++;
    } else {
      runsValid = 0;
    }
  }
}
 
