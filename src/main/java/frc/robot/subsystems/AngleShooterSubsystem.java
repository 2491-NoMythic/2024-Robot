package frc.robot.subsystems;

import frc.robot.settings.Constants.Field;
import frc.robot.settings.Constants.ShooterConstants;
import pabeles.concurrency.IntOperatorTask.Max;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.settings.Constants.ShooterConstants.*;
import static frc.robot.settings.Constants.ShooterConstants.AdjustEquation.*;
import static frc.robot.settings.Constants.LOOPS_VALID_FOR_SHOT;

public class AngleShooterSubsystem extends SubsystemBase {
	CANSparkMax pitchMotor;
	SparkPIDController pitchPID;
	SparkAbsoluteEncoder absoluteEncoder;
	double shootingSpeed = ShooterConstants.SHOOTING_SPEED_MPS;
	public static Pose2d dtvalues;
	public static ChassisSpeeds DTChassisSpeeds;
	public double desiredZeroOffset;
	int runsValid;
	double speakerDistGlobal;
	Field2d offsetSpeakerLocationPose = new Field2d();
	
	public AngleShooterSubsystem() {
		SmartDashboard.putData("offsetSpeakerPose", offsetSpeakerLocationPose);
		
		SmartDashboard.putNumber("CALLIBRATION/redShooterX", Field.CALCULATED_SHOOTER_RED_SPEAKER_X);
		SmartDashboard.putNumber("CALLIBRATION/blueShooterX", Field.CALCULATED_SHOOTER_BLUE_SPEAKER_X);
		SmartDashboard.putNumber("CALLIBRATION/blueY", Field.CALCULATED_BLUE_SPEAKER_Y);
		SmartDashboard.putNumber("CALLIBRATION/redY", Field.CALCULATED_RED_SPEAKER_Y);

		runsValid = 0;
		pitchMotor = new CANSparkMax(PITCH_MOTOR_ID, MotorType.kBrushless);
		pitchMotor.restoreFactoryDefaults();
		pitchMotor.setInverted(true);
		pitchMotor.setIdleMode(IdleMode.kBrake);
		absoluteEncoder = pitchMotor.getAbsoluteEncoder(Type.kDutyCycle);
		absoluteEncoder.setPositionConversionFactor(360);
		absoluteEncoder.setInverted(true);
		if(Preferences.getBoolean("CompBot", false)) {
			absoluteEncoder.setZeroOffset(CompBotZeroOffset);
		} else {
			absoluteEncoder.setZeroOffset(PracBotZeroOffset);
		}


		pitchPID = pitchMotor.getPIDController();
		pitchPID.setFeedbackDevice(absoluteEncoder);
		pitchPID.setP(ANGLE_SHOOTER_POWER_KP);
		pitchPID.setI(ANGLE_SHOOTER_POWER_KI);
		pitchPID.setD(ANGLE_SHOOTER_POWER_KD);
		pitchPID.setOutputRange(pitchMinOutput, pitchMaxOutput);
		pitchMotor.burnFlash();
	}
	/**
	 * sets the setpoint for the pitch motor's onboard PID loop to be this angle. It should make the whole indexer and shooter snap to that angle. 
	 * <p>
	 * 0 degrees is parallel with the ground
	 * @param degrees
	 * the desired degrees
	 */
	public void setDesiredShooterAngle(double degrees) {
		double MaxAngle = COMP_MAXIMUM_SHOOTER_ANGLE;
		if(!Preferences.getBoolean("CompBot", true)) {
			MaxAngle = PRAC_MAXIMUM_SHOOTER_ANGLE;
		}
		if(degrees>MaxAngle) {degrees = COMP_MAXIMUM_SHOOTER_ANGLE;}
		if(degrees<MINIMUM_SHOOTER_ANGLE) {degrees = MINIMUM_SHOOTER_ANGLE;}
		pitchPID.setFF(Math.cos(Math.toRadians(degrees))*ShooterConstants.pitchFeedForward);
		pitchPID.setReference(
			degrees,
			CANSparkMax.ControlType.kPosition
		);
	}
	/**
	 * sets the pitch motor to a percent of it's full power. 
	 * @param pitchSpeed
	 * Max: 1 Min: -1
	 */
	public void pitchShooter(double pitchSpeed) {
		pitchMotor.set(pitchSpeed);
	}
	/**
	 * stops the indexer immediately
	 */
	public void stop() {
		pitchPID.setReference(0, ControlType.kCurrent);
	}
	/**
	 * @return
	 * the current degrees of the shooter and indexer
	 * <p>
	 * 0 degrees is parallel with the ground
	 */
	public double getShooterAngle() {
		return absoluteEncoder.getPosition();// * ShooterConstants.DEGREES_PER_ROTATION;
	}
	/**
	 * sets the dtvalues variable inside the AngleShooterSubsytem
	 * @param pose
	 * the current Pose2d of the robot's location, from the odometry
	 */
	public static void setDTPose(Pose2d pose) {
		dtvalues = pose;
	}
	/**
	 * sets the DTChassisSpeeds variable inside the AngleShooterSubsytem
	 * @param speeds
	 * the current ChassisSpeeds of the drivetrain, gotten using drivetrain.getChassisSpeeds()
	 */
	public static void setDTChassisSpeeds(ChassisSpeeds speeds) {
		DTChassisSpeeds = speeds;
	}
	/**
	 * a math method that will use trigonometry and other math to calculate the current anlge that the shooter should be at to point at the speaker
	 * <p> 
	 * this method uses the variables inside the AngleShooterSubsytem: DTChassisSpeeds and dtvalues, so make sure you set those values before running this method
	 * <p>
	 * you can set those values using {@code setDTPose(Pose2d)} and {@code SetDTChassisSpeeds(ChassisSpeeds)}
	 * @return
	 * the desired angle, in degrees, for the shooter
	 */
	public double calculateSpeakerAngle() {
		double deltaX;
		double deltaY;
		shootingSpeed = ShooterConstants.SHOOTING_SPEED_MPS;
		// triangle for robot angle
		Optional<Alliance> alliance = DriverStation.getAlliance();
		if (alliance.isPresent() && alliance.get() == Alliance.Red) {
			deltaX = Math.abs(dtvalues.getX() - Field.ROBOT_RED_SPEAKER_X);
			deltaY = Math.abs(dtvalues.getY() - Field.RED_SPEAKER_Y);
		} else {
			deltaX = Math.abs(dtvalues.getX() - Field.ROBOT_BLUE_SPEAKER_X);
			deltaY = Math.abs(dtvalues.getY() - Field.BLUE_SPEAKER_Y);
		}
		double speakerDist = Math.sqrt(Math.pow(deltaY, 2) + Math.pow(deltaX, 2));
		// SmartDashboard.putNumber("dist to speaker", speakerDist);
		Rotation2d unadjustedAngle = Rotation2d.fromRadians(Math.asin(deltaX/speakerDist));
		double totalDistToSpeaker = Math.sqrt(Math.pow(Field.SPEAKER_Z-ShooterConstants.SHOOTER_HEIGHT, 2) + Math.pow(speakerDist, 2));
		double shootingTime = totalDistToSpeaker / shootingSpeed; // calculates how long the note will take to reach the target
		double currentXSpeed = DTChassisSpeeds.vxMetersPerSecond;
		double currentYSpeed = DTChassisSpeeds.vyMetersPerSecond;
		Translation2d targetOffset = new Translation2d(currentXSpeed * shootingTime*OFFSET_MULTIPLIER * unadjustedAngle.getRadians(), currentYSpeed * shootingTime*OFFSET_MULTIPLIER * unadjustedAngle.getRadians());
		// line above calculates how much our current speed will affect the ending
		// location of the note if it's in the air for ShootingTime
		
		// next 3 lines set where we actually want to aim, given the offset our shooting
		// will have based on our speed
		double offsetSpeakerX;
		double offsetSpeakerY;
		if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
			offsetSpeakerX = Field.SHOOTER_RED_SPEAKER_X - targetOffset.getX();
			offsetSpeakerY = Field.RED_SPEAKER_Y - targetOffset.getY();
		} else {
			offsetSpeakerX = Field.SHOOTER_BLUE_SPEAKER_X + targetOffset.getX();
			offsetSpeakerY = Field.BLUE_SPEAKER_Y - targetOffset.getY();
		}
		double offsetDeltaX = Math.abs(dtvalues.getX() - offsetSpeakerX);
		double offsetDeltaY = Math.abs(dtvalues.getY() - offsetSpeakerY);
		double offsetSpeakerdist = Math.sqrt(Math.pow(offsetDeltaX, 2) + Math.pow(offsetDeltaY, 2));
		offsetSpeakerdist = offsetSpeakerdist+0.127; //to compensate for the pivot point of the shooter bieng offset from the center of the robot
		SmartDashboard.putString("offset amount", targetOffset.toString());
		Translation2d offsetSpeakerLocation = new Translation2d(offsetSpeakerX, offsetSpeakerY);
		SmartDashboard.putString("offset speaker location", offsetSpeakerLocation.toString());
		offsetSpeakerLocationPose.setRobotPose(new Pose2d(offsetSpeakerLocation, new Rotation2d(0)));
		// getting desired robot angle
		double totalOffsetDistToSpeaker = Math
				.sqrt(Math.pow(offsetSpeakerdist, 2) + Math.pow(Field.SPEAKER_Z - ShooterConstants.SHOOTER_HEIGHT, 2));
		SmartDashboard.putNumber("MATH/shooter's speaker dist", totalOffsetDistToSpeaker);
		double desiredShooterAngle = Math
				.toDegrees(Math.asin((Field.SPEAKER_Z - ShooterConstants.SHOOTER_HEIGHT) / totalOffsetDistToSpeaker));
		desiredShooterAngle = adjustAngleForDistance(desiredShooterAngle, offsetSpeakerdist);
		if(desiredShooterAngle<ShooterConstants.MINIMUM_SHOOTER_ANGLE) {
			desiredShooterAngle = ShooterConstants.MINIMUM_SHOOTER_ANGLE;
		}
		if(desiredShooterAngle>ShooterConstants.PRAC_MAXIMUM_SHOOTER_ANGLE) {
			desiredShooterAngle = ShooterConstants.PRAC_MAXIMUM_SHOOTER_ANGLE;
		}
		if(offsetSpeakerdist>Field.MAX_SHOOTING_DISTANCE) {
			desiredShooterAngle = SAFE_SHOOTER_ANGLE;
		}
		SmartDashboard.putNumber("desired shooter angle", desiredShooterAngle);
		
		
		speakerDistGlobal = offsetSpeakerdist;
		// double differenceAngle = (desiredShooterAngle - this.getShooterAngle());
		// SmartDashboard.putNumber("differenceAngleShooter", differenceAngle);
		
		return desiredShooterAngle;
	}

	public double calculateSpeakerAngleWithSuppliers(DoubleSupplier redShooterXSup, DoubleSupplier blueShooterXSup, DoubleSupplier redYSup, DoubleSupplier blueYSup) {
		double deltaX;
		double deltaY;
		double redX = redShooterXSup.getAsDouble();
		double blueX = blueShooterXSup.getAsDouble();
		double redY = redYSup.getAsDouble();
		double blueY = blueYSup.getAsDouble();
		shootingSpeed = ShooterConstants.SHOOTING_SPEED_MPS;
		// triangle for robot angle
		Optional<Alliance> alliance = DriverStation.getAlliance();
		if (alliance.isPresent() && alliance.get() == Alliance.Red) {
			deltaX = Math.abs(dtvalues.getX() - redX);
			deltaY = Math.abs(dtvalues.getY() - redY);
		} else {
			deltaX = Math.abs(dtvalues.getX() - blueX);
			deltaY = Math.abs(dtvalues.getY() - blueY);
		}
		double speakerDist = Math.sqrt(Math.pow(deltaY, 2) + Math.pow(deltaX, 2));
		// SmartDashboard.putNumber("dist to speaker", speakerDist);
		Rotation2d unadjustedAngle = Rotation2d.fromRadians(Math.asin(deltaX/speakerDist));
		double totalDistToSpeaker = Math.sqrt(Math.pow(Field.SPEAKER_Z-ShooterConstants.SHOOTER_HEIGHT, 2) + Math.pow(speakerDist, 2));
		double shootingTime = totalDistToSpeaker / shootingSpeed; // calculates how long the note will take to reach the target
		double currentXSpeed = DTChassisSpeeds.vxMetersPerSecond;
		double currentYSpeed = DTChassisSpeeds.vyMetersPerSecond;
		Translation2d targetOffset = new Translation2d(currentXSpeed * shootingTime*OFFSET_MULTIPLIER * unadjustedAngle.getRadians(), currentYSpeed * shootingTime*OFFSET_MULTIPLIER * unadjustedAngle.getRadians());
		// line above calculates how much our current speed will affect the ending
		// location of the note if it's in the air for ShootingTime
		
		// next 3 lines set where we actually want to aim, given the offset our shooting
		// will have based on our speed
		double offsetSpeakerX;
		double offsetSpeakerY;
		if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
			offsetSpeakerX = redX - targetOffset.getX();
			offsetSpeakerY = redY - targetOffset.getY();
		} else {
			offsetSpeakerX = redX - targetOffset.getX();
			offsetSpeakerY = redY - targetOffset.getY();
		}
		double offsetDeltaX = Math.abs(dtvalues.getX() - offsetSpeakerX);
		double offsetDeltaY = Math.abs(dtvalues.getY() - offsetSpeakerY);
		double offsetSpeakerdist = Math.sqrt(Math.pow(offsetDeltaX, 2) + Math.pow(offsetDeltaY, 2));
		offsetSpeakerdist = offsetSpeakerdist+0.127; //to compensate for the pivot point of the shooter bieng offset from the center of the robot
		SmartDashboard.putString("offset amount", targetOffset.toString());
		SmartDashboard.putString("offset speaker location",
		new Translation2d(offsetSpeakerX, offsetSpeakerY).toString());
		// getting desired robot angle
		double totalOffsetDistToSpeaker = Math
				.sqrt(Math.pow(offsetSpeakerdist, 2) + Math.pow(Field.SPEAKER_Z - ShooterConstants.SHOOTER_HEIGHT, 2));
		SmartDashboard.putNumber("MATH/shooter's speaker dist", totalOffsetDistToSpeaker);
		double desiredShooterAngle = Math
				.toDegrees(Math.asin((Field.SPEAKER_Z - ShooterConstants.SHOOTER_HEIGHT) / totalOffsetDistToSpeaker));
		desiredShooterAngle = adjustAngleForDistance(desiredShooterAngle, offsetSpeakerdist);
		if(desiredShooterAngle<ShooterConstants.MINIMUM_SHOOTER_ANGLE) {
			desiredShooterAngle = ShooterConstants.MINIMUM_SHOOTER_ANGLE;
		}
		if(desiredShooterAngle>ShooterConstants.PRAC_MAXIMUM_SHOOTER_ANGLE) {
			desiredShooterAngle = ShooterConstants.PRAC_MAXIMUM_SHOOTER_ANGLE;
		}
		if(offsetSpeakerdist>Field.MAX_SHOOTING_DISTANCE) {
			desiredShooterAngle = SAFE_SHOOTER_ANGLE;
		}		
		
		speakerDistGlobal = offsetSpeakerdist;
		// double differenceAngle = (desiredShooterAngle - this.getShooterAngle());
		// SmartDashboard.putNumber("differenceAngleShooter", differenceAngle);
		
		return desiredShooterAngle;
	}
	/**
	 * using the constants made for this, it adjusts our shooter angle based on how far away we are from the speaker
	 * @param initialAngle the angle that the {@code calculateSpeakerAngle()} method calculates before using this
	 * @param distance the robot's distance from the speaker
	 * @return the adjusted angle for the shooter
	 */
	private double adjustAngleForDistance(double initialAngle, double distance) {
		double AdjustEquationA = PRAC_ADJUST_EQUATION_A;
		double AdjustEquationB = PRAC_ADJUST_EQUATION_B;
		double AdjustEquationC = 0;
		double AdjustEquationD = 0;
		if(Preferences.getBoolean("CompBot", true)) {
			AdjustEquationB = COMP_ADJUST_EQUATION_B;
			AdjustEquationA = COMP_ADJUST_EQUATION_A;
			AdjustEquationC = COMP_ADJUST_EQUATION_C;
			AdjustEquationD = COMP_ADJUST_EQUATION_D;
		}
		if(Preferences.getBoolean("CompBot", false)) {
			double errorMeters = AdjustEquationA*Math.pow(distance, 3) + AdjustEquationB*Math.pow(distance, 2) + AdjustEquationC*distance + AdjustEquationD;
			return initialAngle + Math.toDegrees(Math.atan(errorMeters/distance));
		} else {
			double errorMeters = Math.pow(AdjustEquationA, distance) + AdjustEquationB;
			if (errorMeters>0) {
				return initialAngle + Math.toDegrees(Math.atan(errorMeters/distance));
			} else {
				return initialAngle;
			}
		}
	}
	/**
	 * determines if we are close enough to the speaker to use our slower shooting speed
	 * @return if we are in fact close enough
	 */
	public boolean shortSpeakerDist() {
		return speakerDistGlobal<=Field.SHORT_RANGE_SHOOTING_DIST;
	}

	private double calculateSpeakerAngleDifference() {
		return Math.abs(calculateSpeakerAngle() - this.getShooterAngle());
	}
	/**
	 * checks if the shooter has been at the correct angle for more than 20 (from constants) command scheduler loops
	 * @return has it been correct for long enough
	 */
	public boolean validShot() {
		return runsValid >= LOOPS_VALID_FOR_SHOT;
	}
	
	@Override
	public void periodic() {
		// SmartDashboard.putNumber("ANGLE SHOOTER shooter angle encoder position", absoluteEncoder.getPosition()*360);
		SmartDashboard.putNumber("ANGLE SHOOTER absolute encoder value", absoluteEncoder.getPosition());
		// desiredZeroOffset = SmartDashboard.getNumber("ANGLE SHOOTER encoder zero offset", absoluteEncoder.getZeroOffset());
		SmartDashboard.putNumber("ANGLE SHOOTER encoder zero offset", absoluteEncoder.getZeroOffset());

		SmartDashboard.putNumber("ANGLE SHOOTER speaker angle error", calculateSpeakerAngleDifference());

		SmartDashboard.putNumber("CALLIBRATION/Supplied calculated shooter angle", calculateSpeakerAngleWithSuppliers(()->SmartDashboard.getNumber("CALLIBRATION/redShooterX", 0), ()->SmartDashboard.getNumber("CALLIBRATION/blueShooterX", 0), ()->SmartDashboard.getNumber("CALLIBRATION/redY", 0), ()->SmartDashboard.getNumber("CALLIBRATION/blueY", 0)));
		if(calculateSpeakerAngleDifference()<ShooterConstants.ALLOWED_ANGLE_ERROR) {
			runsValid++;
		} else {
			runsValid = 0;
		}
	}
}