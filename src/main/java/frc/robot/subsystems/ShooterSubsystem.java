 // Copyright (c) FIRST and other WPILib contributors.
 // Open Source Software; you can modify and/or share it under the terms of
 // the WPILib BSD license file in the root directory of this project.
 
 package frc.robot.subsystems;
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
   CANSparkMax shooterR;
   CANSparkMax shooterL;
   double runSpeed;

  double differenceAngle;
	 double currentHeading;
	 double m_DesiredShooterAngle;
 
   SparkPIDController shooterPID;
   SparkPIDController pitchPID;
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
     SparkPIDController shooterPID;
     shooterR = new CANSparkMax(ShooterConstants.SHOOTER_R_MOTORID, MotorType.kBrushless);
     shooterL = new CANSparkMax(ShooterConstants.SHOOTER_L_MOTORID, MotorType.kBrushless);
     shooterR.restoreFactoryDefaults();
     shooterL.follow(shooterR);
     shooterL.setInverted(true);
     shooterR.setIdleMode(IdleMode.kCoast);
     shooterL.setIdleMode(IdleMode.kCoast);
     
 
     encoder1 = shooterR.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 4096);
 
    shooterPID = shooterR.getPIDController();
  
    pitchPID.setFF(pitchFeedForward);
 
    shooterPID.setP(kP);
    shooterPID.setI(kI);
    shooterPID.setD(kD);  
    shooterPID.setIZone(kIz);
    shooterPID.setFF(kFF);
    shooterPID.setOutputRange(kMinOutput, kMaxOutput);  
 
 
     //Adjust the value runSpeed later. (SmartDashboard stuff)
     shooterPID.setReference(runSpeed, ControlType.kVelocity);
     
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
 
 
     if((p != kP)) {shooterPID.setP(p); kP = p; }
     if((i != kI)) {shooterPID.setI(i); kI = i; }
     if((d != kD)) {shooterPID.setD(d); kD = d; }
 
     if((iz != kIz)) {shooterPID.setIZone(iz); kIz = iz;}
     if((ff != kFF)) {shooterPID.setFF(ff); kFF = ff;}
     if((max != kMaxOutput) || (min != kMinOutput))
     {
      shooterPID.setOutputRange(min, max);
      kMinOutput = min; kMaxOutput = max;
     }
 
     SmartDashboard.putNumber("Process Variable", encoder1.getPosition());
     
   }
   

  public void shootThing(double runSpeed) {
     shooterR.set(runSpeed);
     shooterL.set(runSpeed);
   }
  public double getError() {
    return Math.abs(shooterR.getAbsoluteEncoder(Type.kDutyCycle).getVelocity()-ShooterConstants.RUNNING_VELOCITY_RPS);
  }
  public void turnOff(){
     shooterR.set(0);
     shooterL.set(0);
  }
}
 
