package frc.robot.helpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.settings.Constants;

import static frc.robot.settings.Constants.*;
import static frc.robot.settings.Constants.ShooterConstants.OFFSET_MULTIPLIER;

import java.util.Optional;

public class MythicalMath {
	/**
	 * clamps an input between a minimum and a maximum. If the input is greater than the maximum, the maximum will
	 * be returned. Vice versa for the minimum.
	 */
	public static double clamp(double input, double maximum, double minimum) {
		return Math.min(Math.max(input, minimum), maximum);
	}

	public static Rotation2d calculateRawAngleToTarget(Pose3d targetCoords, Pose3d robotCoords) {
		Pose3d dtvalues = robotCoords;
		double deltaX = Math.abs(dtvalues.getX() - targetCoords.getX());
		double deltaY = Math.abs(dtvalues.getY() - targetCoords.getY());
		double DistToTarget2D = Math.sqrt(deltaX*deltaX + deltaY*deltaY);
//		double DistToTarget3D = Math.sqrt(Math.pow(targetCoords.getZ() - dtvalues.getZ(), 2) + Math.pow(DistToTarget2D, 2));
		return Rotation2d.fromRadians(Math.asin(deltaX/DistToTarget2D));
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
}