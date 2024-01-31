package frc.robot.subsystems;

import frc.robot.settings.Constants.Field;
import frc.robot.settings.Constants.ShooterConstants;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AngleShooterSubsystem extends SubsystemBase {
	CANSparkMax pitchMotor;
	SparkPIDController pitchPID;
	CANcoder angleEncoder;
	double shootingSpeed = ShooterConstants.SHOOTING_SPEED_MPS;
	public static Pose2d dtvalues;
	public static ChassisSpeeds DTChassisSpeeds;

	public AngleShooterSubsystem() {
		pitchMotor = new CANSparkMax(ShooterConstants.PITCH_MOTOR_ID, MotorType.kBrushless);

		pitchPID = pitchMotor.getPIDController();

		pitchPID.setFF(ShooterConstants.pitchFeedForward);
	}

	public void pitchShooter(double pitchSpeed) {
		pitchMotor.set(pitchSpeed);
	}

	public double getShooterAngle() {
		return angleEncoder.getPosition().getValueAsDouble() * ShooterConstants.DEGREES_PER_ROTATION;
	}

	public static void setDTPose(Pose2d pose) {
		dtvalues = pose;
	}

	public static void setDTChassisSpeeds(ChassisSpeeds speeds) {
		DTChassisSpeeds = speeds;
	}

	public double calculateSpeakerAngle() {
		double deltaY;
		shootingSpeed = ShooterConstants.SHOOTING_SPEED_MPS;
		// triangle for robot angle
		Optional<Alliance> alliance = DriverStation.getAlliance();
		if (alliance.isPresent() && alliance.get() == Alliance.Red) {
			deltaY = Math.abs(dtvalues.getY() - Field.RED_SPEAKER_Y);
		} else {
			deltaY = Math.abs(dtvalues.getY() - Field.BLUE_SPEAKER_Y);
		}
		double deltaX = Math.abs(dtvalues.getX() - Field.SPEAKER_X);
		double speakerDist = Math.sqrt(Math.pow(deltaY, 2) + Math.pow(deltaX, 2));
		SmartDashboard.putNumber("dist to speakre", speakerDist);

		double shootingTime = speakerDist / shootingSpeed; // calculates how long the note will take to reach the target
		double currentXSpeed = DTChassisSpeeds.vxMetersPerSecond;
		double currentYSpeed = DTChassisSpeeds.vyMetersPerSecond;
		Translation2d targetOffset = new Translation2d(currentXSpeed * shootingTime, currentYSpeed * shootingTime);
		// line above calculates how much our current speed will affect the ending
		// location of the note if it's in the air for ShootingTime

		// next 3 lines set where we actually want to aim, given the offset our shooting
		// will have based on our speed
		double offsetSpeakerX = Field.SPEAKER_X + targetOffset.getX();
		double offsetSpeakerY;

		if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
			offsetSpeakerY = Field.RED_SPEAKER_Y + targetOffset.getY();
		} else {
			offsetSpeakerY = Field.BLUE_SPEAKER_Y + targetOffset.getY();
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
				.toDegrees(Math.asin(Field.SPEAKER_Z - ShooterConstants.SHOOTER_HEIGHT / totalDistToSpeaker));

		SmartDashboard.putNumber("desired shooter angle", desiredShooterAngle);

		double differenceAngle = (desiredShooterAngle - this.getShooterAngle());
		SmartDashboard.putNumber("differenceAngleShooter", differenceAngle);

		return differenceAngle;
	}

	
}