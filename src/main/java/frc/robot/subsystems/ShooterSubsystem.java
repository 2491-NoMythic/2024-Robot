 // Copyright (c) FIRST and other WPILib contributors.
 // Open Source Software; you can modify and/or share it under the terms of
 // the WPILib BSD license file in the root directory of this project.
 
 package frc.robot.subsystems;
 import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
 import com.revrobotics.RelativeEncoder;
 import com.revrobotics.SparkPIDController;
 import com.revrobotics.SparkRelativeEncoder;
 import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.commands.AngleShooter;
import frc.robot.commands.RotateRobot;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.Field;
import  frc.robot.settings.Constants.ShooterConstants;
 import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 import edu.wpi.first.wpilibj2.command.SubsystemBase;
 import edu.wpi.first.hal.can.CANStreamOverflowException;
 import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import static frc.robot.settings.Constants.ShooterConstants.*;

import java.util.Optional;
import java.util.function.BooleanSupplier;
 
 public class ShooterSubsystem extends SubsystemBase {
   CANSparkMax shooter1;
   CANSparkMax shooter2;
   CANSparkMax pitchMotor;
   double runSpeed;

  double differenceAngle;
	 double currentHeading;
	 double speakerDist;
	 double deltaY;
	 double deltaX;
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
   CANcoder angleEncoder;

   //speaker angle calculating variables:

	double turningSpeed;
	RotateRobot rotateRobot;
	AngleShooter angleShooter;
	int accumulativeTurns;
	double shootingSpeed = ShooterConstants.SHOOTING_SPEED_MPS;
	double shootingTime; 
	double currentXSpeed;
  public static Pose2d dtvalues;
  public static ChassisSpeeds DTChassisSpeeds;
	double currentYSpeed;
	Translation2d targetOffset; 
	double offsetSpeakerX;
	double offsetSpeakerY;
	Translation2d adjustedTarget;
	double offsetSpeakerdist;
  BooleanSupplier aimAtAmp;
 
     /** Creates a new Shooter. */
   public ShooterSubsystem(double runSpeed, BooleanSupplier aimAtAmp) {
     SparkPIDController shooterPID;
     this.aimAtAmp = aimAtAmp;
     shooter1 = new CANSparkMax(ShooterConstants.SHOOTER_1_MOTORID, MotorType.kBrushless);
     shooter2 = new CANSparkMax(ShooterConstants.SHOOTER_2_MOTORID, MotorType.kBrushless);
     pitchMotor = new CANSparkMax(ShooterConstants.PITCH_MOTOR_ID, MotorType.kBrushless);
     shooter1.restoreFactoryDefaults();
     shooter2.follow(shooter1);
     shooter2.setInverted(true);
     shooter1.setIdleMode(IdleMode.kCoast);
     shooter2.setIdleMode(IdleMode.kCoast);
     
 
     encoder1 = shooter1.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 4096);
 
    shooterPID = shooter1.getPIDController();
    pitchPID = pitchMotor.getPIDController();              
  
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
     shooter1.set(runSpeed);
   }
  public void turnOff(){
     shooter1.set(0);
   }
   public void pitchShooter(double pitchSpeed){
    pitchMotor.set(pitchSpeed);
  }
  public double getShooterAngle(){
    return angleEncoder.getPosition().getValueAsDouble()*DEGREES_PER_ROTATION;
  }
  public static void setDTPose(Pose2d pose) {
    dtvalues = pose;
  }
  public static void setDTChassisSpeeds(ChassisSpeeds speeds) {
    DTChassisSpeeds = speeds;
  }
  public double calculateSpeakerAngle() {
		shootingSpeed = ShooterConstants.SHOOTING_SPEED_MPS;
		//triangle for robot angle
		Optional<Alliance> alliance = DriverStation.getAlliance();
		if (alliance.isPresent() && alliance.get() == Alliance.Red) {
			deltaY = Math.abs(dtvalues.getY() - Field.RED_SPEAKER_Y);
		} else {
			deltaY = Math.abs(dtvalues.getY() - Field.BLUE_SPEAKER_Y);
		}
		deltaX = Math.abs(dtvalues.getX() - Field.SPEAKER_X);
		speakerDist = Math.sqrt(Math.pow(deltaY, 2) + Math.pow(deltaX, 2));
		SmartDashboard.putNumber("dist to speakre", speakerDist);
	
		shootingTime = speakerDist/shootingSpeed; //calculates how long the note will take to reach the target
		currentXSpeed = DTChassisSpeeds.vxMetersPerSecond;
		currentYSpeed = DTChassisSpeeds.vyMetersPerSecond;
		targetOffset = new Translation2d(currentXSpeed*shootingTime, currentYSpeed*shootingTime); 
		//line above calculates how much our current speed will affect the ending location of the note if it's in the air for ShootingTime
		
		//next 3 lines set where we actually want to aim, given the offset our shooting will have based on our speed
		offsetSpeakerX = Field.SPEAKER_X+targetOffset.getX();
		if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {offsetSpeakerY = Field.RED_SPEAKER_Y+targetOffset.getY();}
		else {offsetSpeakerY = Field.BLUE_SPEAKER_Y+targetOffset.getY();}
		double offsetDeltaX = Math.abs(dtvalues.getX() - offsetSpeakerX);
		double offsetDeltaY = Math.abs(dtvalues.getY() - offsetSpeakerY);
		offsetSpeakerdist = Math.sqrt(Math.pow(offsetDeltaX, 2) + Math.pow(offsetDeltaY, 2));
		SmartDashboard.putString("offset amount", targetOffset.toString());
		SmartDashboard.putString("offset speaker location", new Translation2d(offsetSpeakerX, offsetSpeakerY).toString());
		//getting desired robot angle
		double totalDistToSpeaker = Math.sqrt(Math.pow(offsetSpeakerdist, 2) + Math.pow(Field.SPEAKER_Z-SHOOTER_HEIGHT, 2));
		double desiredShooterAngle = Math.toDegrees(Math.asin(Field.SPEAKER_Z-SHOOTER_HEIGHT / totalDistToSpeaker));		
		
		SmartDashboard.putNumber("desired shooter angle", desiredShooterAngle);

    differenceAngle = (desiredShooterAngle - this.getShooterAngle());
    SmartDashboard.putNumber("differenceAngleShooter", differenceAngle);

    return differenceAngle;
  }

@Override
  public void periodic() {
    if (!aimAtAmp.getAsBoolean()) {
    double desiredShooterAngleSpeed = calculateSpeakerAngle()*ShooterConstants.AUTO_AIM_SHOOTER_kP;
    pitchShooter(desiredShooterAngleSpeed);
  }
   else {
    double differenceAmp = Field.AMPLIFIER_ANGLE-this.getShooterAngle();
    double ampSpeed = differenceAmp*ShooterConstants.AUTO_AIM_SHOOTER_kP;
    pitchShooter(ampSpeed);
  }
  }
}
 
