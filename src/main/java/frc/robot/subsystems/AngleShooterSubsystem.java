package frc.robot.subsystems;

import frc.robot.settings.Constants.Field;
import frc.robot.settings.Constants.ShooterConstants;

import static frc.robot.settings.Constants.ShooterConstants.DISTANCE_MULTIPLIER;
import static frc.robot.settings.Constants.ShooterConstants.OFFSET_MULTIPLIER;
import static frc.robot.settings.Constants.ShooterConstants.ROBOT_ANGLE_TOLERANCE;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AngleShooterSubsystem extends SubsystemBase {
	CANSparkMax pitchMotor;
	SparkPIDController pitchPID;
	SparkAbsoluteEncoder absoluteEncoder;
	double shootingSpeed = ShooterConstants.SHOOTING_SPEED_MPS;
	public static Pose2d dtvalues;
	public static ChassisSpeeds DTChassisSpeeds;
	public double desiredZeroOffset;

	public AngleShooterSubsystem() {
		pitchMotor = new CANSparkMax(ShooterConstants.PITCH_MOTOR_ID, MotorType.kBrushless);
		pitchMotor.setInverted(true);
		pitchMotor.setIdleMode(IdleMode.kBrake);
		pitchPID = pitchMotor.getPIDController();
		pitchPID.setFF(ShooterConstants.pitchFeedForward);
		absoluteEncoder = pitchMotor.getAbsoluteEncoder(Type.kDutyCycle);
		absoluteEncoder.setPositionConversionFactor(1);
		absoluteEncoder.setInverted(true);
		desiredZeroOffset = 38;//absoluteEncoder.getZeroOffset();
		absoluteEncoder.setZeroOffset(Math.toRadians(desiredZeroOffset));
		pitchMotor.burnFlash();
		SmartDashboard.putData("set zero offset", new InstantCommand(()->absoluteEncoder.setZeroOffset(desiredZeroOffset)));
		SmartDashboard.putNumber("shooter encoder zero offset", desiredZeroOffset/*absoluteEncoder.getZeroOffset()*/);
	}
	
	public void pitchShooter(double pitchSpeed) {
		pitchMotor.set(pitchSpeed);
	}
	
	public double getShooterAngle() {
		return Math.toDegrees(absoluteEncoder.getPosition());// * ShooterConstants.DEGREES_PER_ROTATION;
	}
	
	public static void setDTPose(Pose2d pose) {
		dtvalues = pose;
	}
	
	public static void setDTChassisSpeeds(ChassisSpeeds speeds) {
		DTChassisSpeeds = speeds;
	}
	
	public double calculateSpeakerAngleDifference() {
		double deltaX;
		shootingSpeed = ShooterConstants.SHOOTING_SPEED_MPS;
		// triangle for robot angle
		Optional<Alliance> alliance = DriverStation.getAlliance();
		if (alliance.isPresent() && alliance.get() == Alliance.Red) {
			deltaX = Math.abs(dtvalues.getX() - Field.RED_SPEAKER_X);
		} else {
			deltaX = Math.abs(dtvalues.getX() - Field.BLUE_SPEAKER_X);
		}
		double deltaY = Math.abs(dtvalues.getY() - Field.SPEAKER_Y);
		double speakerDist = Math.sqrt(Math.pow(deltaY, 2) + Math.pow(deltaX, 2));
		SmartDashboard.putNumber("dist to speakre", speakerDist);
		
		double shootingTime = speakerDist / shootingSpeed; // calculates how long the note will take to reach the target
		double currentXSpeed = DTChassisSpeeds.vxMetersPerSecond;
		double currentYSpeed = DTChassisSpeeds.vyMetersPerSecond;
		Translation2d targetOffset = new Translation2d(currentXSpeed * shootingTime*OFFSET_MULTIPLIER, currentYSpeed * shootingTime*OFFSET_MULTIPLIER);
		// line above calculates how much our current speed will affect the ending
		// location of the note if it's in the air for ShootingTime
		
		// next 3 lines set where we actually want to aim, given the offset our shooting
		// will have based on our speed
		double offsetSpeakerY = Field.SPEAKER_Y + targetOffset.getY();
		double offsetSpeakerX;
		
		if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
			offsetSpeakerX = Field.RED_SPEAKER_X + targetOffset.getX();
		} else {
			offsetSpeakerX = Field.BLUE_SPEAKER_X + targetOffset.getX();
		}
		double offsetDeltaX = Math.abs(dtvalues.getX() - offsetSpeakerX);
		double offsetDeltaY = Math.abs(dtvalues.getY() - offsetSpeakerY);
		double offsetSpeakerdist = Math.sqrt(Math.pow(offsetDeltaX, 2) + Math.pow(offsetDeltaY, 2));
		SmartDashboard.putString("offset amount", targetOffset.toString());
		SmartDashboard.putString("offset speaker location",
		new Translation2d(offsetSpeakerX, offsetSpeakerY).toString());
		// getting desired robot angle
		double totalDistToSpeaker = Math
				.sqrt(Math.pow(offsetSpeakerdist, 2) + Math.pow(Field.SPEAKER_Z - ShooterConstants.SHOOTER_HEIGHT, 2));
		double desiredShooterAngle = Math
				.toDegrees(Math.asin((Field.SPEAKER_Z - ShooterConstants.SHOOTER_HEIGHT) / totalDistToSpeaker));
		desiredShooterAngle = desiredShooterAngle+(speakerDist*DISTANCE_MULTIPLIER);
		if(desiredShooterAngle<ShooterConstants.MINIMUM_SHOOTER_ANGLE) {
			desiredShooterAngle = ShooterConstants.MINIMUM_SHOOTER_ANGLE;
		}
		if(desiredShooterAngle>ShooterConstants.MAXIMUM_SHOOTER_ANGLE) {
			desiredShooterAngle = ShooterConstants.MAXIMUM_SHOOTER_ANGLE;
		}
		SmartDashboard.putNumber("desired shooter angle", desiredShooterAngle);
		
		double differenceAngle = (desiredShooterAngle - this.getShooterAngle());
		SmartDashboard.putNumber("differenceAngleShooter", differenceAngle);
		
		return differenceAngle;
	}
	
	@Override
	public void periodic() {
		// SmartDashboard.putNumber("shooter angle encoder position", Math.toDegrees(absoluteEncoder.getPosition()));
		// desiredZeroOffset = SmartDashboard.getNumber("shooter encoder zero offset", absoluteEncoder.getZeroOffset());
		// SmartDashboard.putNumber("shooter encoder zero offset", absoluteEncoder.getZeroOffset());
	}
}