 // Copyright (c) FIRST and other WPILib contributors.
 // Open Source Software; you can modify and/or share it under the terms of
 // the WPILib BSD license file in the root directory of this project.
 
 package frc.robot.subsystems;
 import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
 import com.revrobotics.RelativeEncoder;
 import com.revrobotics.SparkPIDController;
 import com.revrobotics.SparkRelativeEncoder;
 import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import frc.robot.commands.AngleShooter;
import frc.robot.commands.RotateRobot;
import frc.robot.settings.Constants;
import  frc.robot.settings.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.settings.Constants.ShooterConstants.*;
 
 public class ShooterSubsystem extends SubsystemBase {
   TalonFX shooterR;
   TalonFX shooterL;
   double runSpeed;
   TalonFXConfigurator configuratorR;
   TalonFXConfigurator configuratorL;

  double differenceAngle;
	 double currentHeading;
	 double m_DesiredShooterAngle;
 
   Slot0Configs PIDconfigs = new Slot0Configs();
      double kP = Constants.ShooterConstants.kP;         
   double kI = Constants.ShooterConstants.kI;         
   double kD = Constants.ShooterConstants.kD;         
   double kIz = Constants.ShooterConstants.kIz;         
   double kFF = Constants.ShooterConstants.kFF;         
   double kMaxOutput = Constants.ShooterConstants.kMaxOutput;         
   double kMinOutput = Constants.ShooterConstants.kMinOutput; 
 
   RelativeEncoder encoder1;

   //speaker angle calculating variables:

	double turningSpeed;
	RotateRobot rotateRobot;
	AngleShooter angleShooter;
	int accumulativeTurns;
  
  /** Creates a new Shooter. */
  public ShooterSubsystem(double runSpeed) {
    shooterR = new TalonFX(ShooterConstants.SHOOTER_R_MOTORID);
    shooterL = new TalonFX(ShooterConstants.SHOOTER_L_MOTORID);
    shooterL.setInverted(true);
    shooterR.setInverted(false);
    shooterL.setControl(shooterR.getAppliedControl());
    shooterL.setNeutralMode(NeutralModeValue.Coast);
    shooterR.setNeutralMode(NeutralModeValue.Coast);
    PIDconfigs = new Slot0Configs();
    
    configuratorR = shooterR.getConfigurator();
    configuratorL = shooterL.getConfigurator();
    
    PIDconfigs.kP = kP;
    PIDconfigs.kI = kI;
    PIDconfigs.kD = kD;
    PIDconfigs.kV = kFF;
    
     SmartDashboard.putNumber("P Gain", Constants.ShooterConstants.kP);
     SmartDashboard.putNumber("I Gain", Constants.ShooterConstants.kI);
     SmartDashboard.putNumber("D Gain",Constants.ShooterConstants.kD);
     SmartDashboard.putNumber("I Zone",Constants.ShooterConstants.kIz);
     SmartDashboard.putNumber("Feed Forward",Constants.ShooterConstants.kFF);
     SmartDashboard.putNumber("Max Output",Constants.ShooterConstants.kMaxOutput);
     SmartDashboard.putNumber("Min Output",Constants.ShooterConstants.kMinOutput);
     SmartDashboard.putNumber("Set Velocity", runSpeed);
 
     double p = SmartDashboard.getNumber("P Gain", 0);
     double i = SmartDashboard.getNumber("I Gain", 0);
     double d = SmartDashboard.getNumber("D Gain", 0);
     double iz = SmartDashboard.getNumber("I Zone", 0);
     double ff = SmartDashboard.getNumber("Feed Forward", 0);
     double max = SmartDashboard.getNumber("Max Output", 0);
     double min = SmartDashboard.getNumber("Min Output", 0);   
     double ve = SmartDashboard.getNumber("Set Velocity", 0);
     
     
     if((p != kP)) {PIDconfigs.kP = p; kP = p; }
     if((i != kI)) {PIDconfigs.kI = i; kI = i; }
     if((d != kD)) {PIDconfigs.kD = d; kD = d; }
     
     if((ff != kFF)) {PIDconfigs.kS = ff; kFF = ff;}
     configuratorR.apply(PIDconfigs);
     configuratorL.apply(PIDconfigs);
    }
   
    
    public void shootThing(double runSpeed) {
      shooterR.set(runSpeed);
      shooterL.set(runSpeed);
    }
    public void shootRPS(double RPS) {
      shooterR.setControl(new VelocityDutyCycle(RPS).withSlot(0));
      shooterL.setControl(new VelocityDutyCycle(RPS).withSlot(0));
    }
    private double getError() {
      return Math.abs(shooterR.getClosedLoopError().getValueAsDouble());
    }
    public boolean validShot() {
      return getError()<ShooterConstants.ALLOWED_SPEED_ERROR;
    }
  public void turnOff(){
    shooterR.set(0);
    shooterL.set(0);
  }
  // public double getSpeedRPS() {
  //   return shooterR.getVelocity().asSupplier().get();
  // }
@Override
  public void periodic() {
    SmartDashboard.putNumber("TESTING shooter speed error", getError());
  }
}
 
