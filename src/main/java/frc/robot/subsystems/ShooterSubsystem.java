 // Copyright (c) FIRST and other WPILib contributors.
 // Open Source Software; you can modify and/or share it under the terms of
 // the WPILib BSD license file in the root directory of this project.
 
 package frc.robot.subsystems;
 import com.revrobotics.CANSparkMax;
 import com.revrobotics.RelativeEncoder;
 import com.revrobotics.SparkPIDController;
 import com.revrobotics.SparkRelativeEncoder;
 import com.revrobotics.CANSparkBase.ControlType;
 import com.revrobotics.CANSparkLowLevel.MotorType;
 
 import frc.robot.settings.Constants;
 import  frc.robot.settings.Constants.ShooterConstants;
 import edu.wpi.first.math.controller.PIDController;
 import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 import edu.wpi.first.wpilibj2.command.SubsystemBase;
 import edu.wpi.first.hal.can.CANStreamOverflowException;
 import edu.wpi.first.math.geometry.Pose2d;
 import static frc.robot.settings.Constants.ShooterConstants.*;
 
 public class ShooterSubsystem extends SubsystemBase {
   CANSparkMax shooter1;
   CANSparkMax shooter2;
   CANSparkMax pitchMotor;
   double runSpeed;

  double differenceAngle;
	 double currentHeading;
	 double speakerDist;
	 double speakerA;
	 double speakerB;
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
   UniversalEncoder universalEncoder;

   //speaker angle calculating variables:
	double m_desiredRobotAngle;
	double differenceAngle;
	double currentHeading;
	double speakerDist;
	double speakerA;
	double speakerB;
	double m_DesiredShooterAngle;
	double turningSpeed;
	RotateRobot rotateRobot;
	AngleShooter angleShooter;
	int accumulativeTurns;
	Pose2d dtvalues;
	double shootingSpeed = ShooterConstants.SHOOTING_SPEED_MPS;
	double shootingTime; 
	double currentXSpeed;
	double currentYSpeed;
	Translation2d targetOffset; 
	double offsetSpeakerX;
	double offsetSpeakerY;
	Translation2d adjustedTarget;
	double offsetSpeakerdist;
 
     /** Creates a new Shooter. */
   public ShooterSubsystem(double runSpeed) {
     SparkPIDController shooterPID;
     shooter1 = new CANSparkMax(ShooterConstants.SHOOTER_1_MOTORID, MotorType.kBrushless);
     shooter2 = new CANSparkMax(ShooterConstants.SHOOTER_2_MOTORID, MotorType.kBrushless);
     pitchMotor = new CANSparkMax(ShooterConstants.PITCH_MOTOR_ID, MotorType.kBrushless);
     shooter1.restoreFactoryDefaults();
     shooter2.follow(shooter1);
     shooter2.setInverted(true);
     
 
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
  public void setAngle(double angle){
    pitchPID.setSetpoint(angle*TICKS_PER_DEGREE);
  }
  public void goToSetAngle(){
    pitchMotor.set(pitchPID.calculate(pitchMotor.getVelocityOutput))
  }
  public void getShooterAngle(){
    universalEncoder.getPosition/ANGLE_TICKS_PER_DEGREE;
  }
  public void calculateSpeakerAngle() {
    dtvalues = this.getPose();
		shootingSpeed = ShooterConstants.SHOOTING_SPEED_MPS;
		//triangle for robot angle
		if (DriverStation.getAlliance().equals(Alliance.Red)) {
			speakerA = Math.abs(dtvalues.getX() - Field.RED_SPEAKER_X);
		} else {
			speakerA = Math.abs(dtvalues.getX() - Field.BLUE_SPEAKER_X);
		}
		speakerB = Math.abs(dtvalues.getY() - Field.SPEAKER_Y);
		speakerDist = Math.sqrt(Math.pow(speakerA, 2) + Math.pow(speakerB, 2));
		SmartDashboard.putNumber("dist to speakre", speakerDist);
	
		shootingTime = speakerDist/shootingSpeed; //calculates how long the note will take to reach the target
		currentXSpeed = this.getChassisSpeeds().vxMetersPerSecond;
		currentYSpeed = this.getChassisSpeeds().vyMetersPerSecond;
		targetOffset = new Translation2d(currentXSpeed*shootingTime, currentYSpeed*shootingTime); 
		//line above calculates how much our current speed will affect the ending location of the note if it's in the air for ShootingTime
		
		//next 3 lines set where we actually want to aim, given the offset our shooting will have based on our speed
		offsetSpeakerX = speakerA-targetOffset.getX();
		offsetSpeakerY = speakerB-targetOffset.getY();
		offsetSpeakerdist = Math.sqrt(Math.pow(offsetSpeakerX, 2) + Math.pow(offsetSpeakerY, 2));
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
}
 
