package frc.robot.helpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.Field;
import frc.robot.settings.Constants.ShooterConstants;
import frc.robot.settings.Constants.ShooterConstants.AdjustEquation;

import static frc.robot.settings.Constants.*;
import static frc.robot.settings.Constants.ShooterConstants.OFFSET_MULTIPLIER;

import java.util.Optional;

public class MythicalMath {
	/**
	 * GRAVITY: Acceleration via gravity in meters per second squared. 
	 */
	public static final double GRAVITY = -9.81;
	public static class ShooterConfiguration {
		public double robotAngle;
		public double shooterAngle;
		public ShooterConfiguration(double robotAngle, double shooterAngle) {
			this.robotAngle = robotAngle;
			this.shooterAngle = shooterAngle;
		}
	}

	/**
	 * clamps an input between a minimum and a maximum. If the input is greater than the maximum, the maximum will
	 * be returned. Vice versa for the minimum.
	 */
	public static double clamp(double input, double maximum, double minimum) {
		return Math.min(Math.max(input, minimum), maximum);
	}

	public static Rotation2d calculateRawAngleToTarget(Pose3d targetCoords, Pose3d robotCoords) {
		Pose3d dtvalues = robotCoords;
		double deltaX = dtvalues.getX() - targetCoords.getX();
		double deltaY = dtvalues.getY() - targetCoords.getY();
//		double DistToTarget3D = Math.sqrt(Math.pow(targetCoords.getZ() - dtvalues.getZ(), 2) + Math.pow(DistToTarget2D, 2));
		return Rotation2d.fromRadians(Math.atan2(deltaY, deltaX));
	}

public static double adjustAngleForSpeed(Pose3d robotCoords, ChassisSpeeds robotSpeeds, Rotation2d startingAngle, Optional<DriverStation.Alliance> alliance, double shotSpeed, double distToTarget, Pose3d targetCoords) {
		double shootingTime = distToTarget/shotSpeed;
		Translation2d targetOffset = new Translation2d(robotSpeeds.vxMetersPerSecond*shootingTime*OFFSET_MULTIPLIER*startingAngle.getRadians(), robotSpeeds.vyMetersPerSecond*shootingTime*OFFSET_MULTIPLIER);
		//line above calculates how much our current speed will affect the ending location of the note if it's in the air for ShootingTime

		//next 3 lines set where we actually want to aim, given the offset our shooting will have based on our speed
		int correctionDirection;
		double speakerX;
		if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
			correctionDirection = 1;
			speakerX = Constants.Field.BLUE_SPEAKER_X;
		} else {
			correctionDirection = -1;
			speakerX = Constants.Field.RED_SPEAKER_X;
		}
		double offsetSpeakerX = targetCoords.getX()+(targetOffset.getX()*correctionDirection);
		double offsetSpeakerY = targetCoords.getY()+(targetOffset.getY()*correctionDirection);
		double offsetDeltaX = Math.abs(robotCoords.getX() - offsetSpeakerX);
		double offsetDeltaY = Math.abs(robotCoords.getY() - offsetSpeakerY);

		Translation2d adjustedTarget = new Translation2d(offsetSpeakerX, offsetSpeakerY);
	}

	private double adjustAngleForDistance(double initialAngle, double distance) {
		double AdjustEquationA = AdjustEquation.PRAC_ADJUST_EQUATION_A;
		double AdjustEquationB = AdjustEquation.PRAC_ADJUST_EQUATION_B;
		double AdjustEquationC = 0;
		if(Preferences.getBoolean("CompBot", true)) {
			AdjustEquationB = AdjustEquation.COMP_ADJUST_EQUATION_B;
			AdjustEquationA = AdjustEquation.COMP_ADJUST_EQUATION_A;
			AdjustEquationC = AdjustEquation.COMP_ADJUST_EQUATION_C;
		}
		if(Preferences.getBoolean("CompBot", false)) {
			double errorMeters = AdjustEquationA*Math.pow(distance, 2) + AdjustEquationB*distance + AdjustEquationC;
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

	public double CalculateAngleShooterSpeakerAngle(Pose2d dtvalues, ChassisSpeeds DTChassisSpeeds, double speakerDistGlobal){
		double deltaX;
		double shootingSpeed = ShooterConstants.SHOOTING_SPEED_MPS;
		//triangle for robot angle
		Optional<Alliance> alliance = DriverStation.getAlliance();
		if(alliance.isPresent() && alliance.get() == Alliance.Red){
			deltaX = Math.abs(dtvalues.getX() - Field.RED_SPEAKER_X);
		}
		else{
			deltaX = Math.abs(dtvalues.getX() - Field.BLUE_SPEAKER_X);
		}
		double deltaY = Math.abs(dtvalues.getY() - Field.SPEAKER_Y);
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
			offsetSpeakerX = Field.RED_SPEAKER_X - targetOffset.getX();
			offsetSpeakerY = Field.SPEAKER_Y - targetOffset.getY();
		} else {
			offsetSpeakerX = Field.BLUE_SPEAKER_X - targetOffset.getX();
			offsetSpeakerY = Field.SPEAKER_Y - targetOffset.getY();
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
			desiredShooterAngle = ShooterConstants.SAFE_SHOOTER_ANGLE;
		}
		SmartDashboard.putNumber("desired shooter angle", desiredShooterAngle);
		
		speakerDistGlobal = offsetSpeakerdist;
		// double differenceAngle = (desiredShooterAngle - this.getShooterAngle());
		// SmartDashboard.putNumber("differenceAngleShooter", differenceAngle);
		
		return desiredShooterAngle;
	}

	private double getDistance(double X, double Y){
		return Math.sqrt(Y * Y + X * X);
	}

	/**
	 * Gets the ShooterConfiguration that points directly at the given target
	 * @param target the difference between the robot's position and the target's position
	 */
	public ShooterConfiguration pointAtTarget(Translation3d target){
		double Y = target.getY();
		double X = target.getX();
		double robot = Math.atan2(Y, X);
		double shooter = Math.atan2(target.getZ(), getDistance(X, Y));
		
		return new ShooterConfiguration(robot, shooter);
	}
	/**
	 * Adjusts the target to compensate for the movement of the robot relative to the target and gravity.
	 * @param base The unadjusted target in meters from the robot location 
	 * @param robotVelocity The velocity of the robot in meters per second 
	 * @param time The time the note is expected to take to hit the target in seconds
	 */
	public Translation3d adjustTarget(Translation3d base, Translation2d robotVelocity, double time){
		double x = base.getX();
		double y = base.getY();
		double z = base.getZ();
		x -= robotVelocity.getX() * time;
		y -= robotVelocity.getY() * time;
		z -= (GRAVITY / 2) * (time * time);
		return new Translation3d(x, y, z);
	}

	public ShooterConfiguration pointAtTargetWithAdjustments(Translation3d base, Translation2d robotVelocity, double shooterVelocity) {
		ShooterConfiguration angles = pointAtTarget(base);
		for (int i=0; i<4; i++){
			double horizontalDistance = getDistance(base.getX(),base.getY());
			double horizontalShooterVelocity = Math.cos(angles.shooterAngle)*shooterVelocity;
			double velocityX = horizontalShooterVelocity*Math.cos(angles.robotAngle) + robotVelocity.getX();
			double velocityY = horizontalShooterVelocity*Math.sin(angles.robotAngle) + robotVelocity.getY();
			double horizontalVelocity = getDistance(velocityX, velocityY);
			Translation3d adjusted = adjustTarget(base, robotVelocity, horizontalDistance/horizontalVelocity);
		}
		return angles;
	}
}