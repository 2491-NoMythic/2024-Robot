// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.settings.Constants.DriveConstants.BL_DRIVE_MOTOR_ID;
import static frc.robot.settings.Constants.DriveConstants.BL_STEER_ENCODER_ID;
import static frc.robot.settings.Constants.DriveConstants.BL_STEER_MOTOR_ID;
import static frc.robot.settings.Constants.DriveConstants.BR_DRIVE_MOTOR_ID;
import static frc.robot.settings.Constants.DriveConstants.BR_STEER_ENCODER_ID;
import static frc.robot.settings.Constants.DriveConstants.BR_STEER_MOTOR_ID;
import static frc.robot.settings.Constants.DriveConstants.CANIVORE_DRIVETRAIN;
import static frc.robot.settings.Constants.DriveConstants.DRIVETRAIN_PIGEON_ID;
import static frc.robot.settings.Constants.DriveConstants.DRIVE_ODOMETRY_ORIGIN;
import static frc.robot.settings.Constants.DriveConstants.FL_DRIVE_MOTOR_ID;
import static frc.robot.settings.Constants.DriveConstants.FL_STEER_ENCODER_ID;
import static frc.robot.settings.Constants.DriveConstants.FL_STEER_MOTOR_ID;
import static frc.robot.settings.Constants.DriveConstants.FR_DRIVE_MOTOR_ID;
import static frc.robot.settings.Constants.DriveConstants.FR_STEER_ENCODER_ID;
import static frc.robot.settings.Constants.DriveConstants.FR_STEER_MOTOR_ID;
import static frc.robot.settings.Constants.ShooterConstants.OFFSET_MULTIPLIER;

import java.util.Arrays;
import java.util.Collections;
import java.util.Optional;

import javax.swing.text.html.Option;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.commands.AngleShooter;
import frc.robot.commands.Drive;
import frc.robot.commands.RotateRobot;
import frc.robot.settings.Constants;
import frc.robot.settings.LimelightValues;
import frc.robot.settings.Constants.CTREConfigs;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.settings.Constants.Vision;
import frc.robot.settings.Constants.DriveConstants.Offsets;
import frc.robot.settings.Constants.DriveConstants.Positions;
import frc.robot.settings.Constants.Field;
import frc.robot.settings.Constants.ShooterConstants;

public class DrivetrainSubsystem extends SubsystemBase {
	public static final CTREConfigs ctreConfig = new CTREConfigs();
	public SwerveDriveKinematics kinematics = DriveConstants.kinematics;

	private final Pigeon2 pigeon = new Pigeon2(DRIVETRAIN_PIGEON_ID, CANIVORE_DRIVETRAIN);

	/**
	 * These are our modules. We initialize them in the constructor.
	 * 0 = Front Left
	 * 1 = Front Right
	 * 2 = Back Left
	 * 3 = Back Right
	 */
	private final SwerveModule[] modules;
	private final Rotation2d[] lastAngles;
	private int accumulativeLoops;

	private final SwerveDrivePoseEstimator odometer;
	private final Field2d m_field = new Field2d();

	//speaker angle calculating variables:
	double m_desiredRobotAngle;
	double differenceAngle;
	double currentHeading;
	double deltaX;
	double deltaY;
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
	double offsetDeltaX;
	double offsetDeltaY;
	Translation2d adjustedTarget;
	double offsetSpeakerdist;
	public double speakerDist;
	Limelight limelight;

	double MathRanNumber;

	public DrivetrainSubsystem() {
		MathRanNumber = 0;
		this.limelight=Limelight.getInstance();

		Preferences.initDouble("FL offset", 0);
		Preferences.initDouble("FR offset", 0);
		Preferences.initDouble("BL offset", 0);
		Preferences.initDouble("BR offset", 0);
		PathPlannerLogging.setLogActivePathCallback((poses) -> m_field.getObject("path").setPoses(poses));
		SmartDashboard.putData("Field", m_field);
		SmartDashboard.putData("resetOdometry", new InstantCommand(() -> this.resetOdometry()));
		SmartDashboard.putBoolean("use limelight", false);
		SmartDashboard.putBoolean("trust limelight", false);
		SmartDashboard.putBoolean("force use limelight", false);
		SmartDashboard.putBoolean("use 2 limelights", false);
		modules = new SwerveModule[4];
		lastAngles = new Rotation2d[] {new Rotation2d(), new Rotation2d(), new Rotation2d(), new Rotation2d()}; // manually make empty angles to avoid null errors.

		modules[0] = new SwerveModule(
			"FL",
			FL_DRIVE_MOTOR_ID,
			FL_STEER_MOTOR_ID,
			FL_STEER_ENCODER_ID,
			Rotation2d.fromRotations(Preferences.getDouble("FL offset", 0)),
			CANIVORE_DRIVETRAIN);
		modules[1] = new SwerveModule(
			"FR",
			FR_DRIVE_MOTOR_ID,
			FR_STEER_MOTOR_ID,
			FR_STEER_ENCODER_ID,
			Rotation2d.fromRotations(Preferences.getDouble("FR offset", 0)),
			CANIVORE_DRIVETRAIN);
		modules[2] = new SwerveModule(
			"BL",
			BL_DRIVE_MOTOR_ID,
			BL_STEER_MOTOR_ID,
			BL_STEER_ENCODER_ID,
			Rotation2d.fromRotations(Preferences.getDouble("BL offset", 0)),
			CANIVORE_DRIVETRAIN);
		modules[3] = new SwerveModule(
			"BR",
			BR_DRIVE_MOTOR_ID,
			BR_STEER_MOTOR_ID,
			BR_STEER_ENCODER_ID,
			Rotation2d.fromRotations(Preferences.getDouble("BR offset", 0)),
			CANIVORE_DRIVETRAIN);
		
		odometer = new SwerveDrivePoseEstimator(
			kinematics, 
			getGyroscopeRotation(),
			getModulePositions(),
			DRIVE_ODOMETRY_ORIGIN);
		}
	/**
	 * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
	 * 'forwards' direction.
	 */
	public void zeroGyroscope() {
		if(DriverStation.getAlliance().get() == Alliance.Red) {
			pigeon.setYaw(180);
		} else {
		pigeon.setYaw(0); //TODO make sure this is right for both alliances
		}
		odometer.resetPosition(new Rotation2d(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
	}
	public void zeroGyroscope(double angleDeg) {
		pigeon.setYaw(angleDeg);
		new Rotation2d();
		new Rotation2d();
		odometer.resetPosition(Rotation2d.fromDegrees(angleDeg), getModulePositions(), new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(angleDeg)));
	}
	public double getHeadingLooped() {
		//returns the heading of the robot, but only out of 360, not accumulative
		accumulativeLoops = (int) (getHeadingDegrees()/180); //finding the amount of times that 360 goes into the heading, as an int
		return getHeadingDegrees()-180*(accumulativeLoops); 
	}
	public Rotation2d getGyroscopeRotation() {
		return pigeon.getRotation2d();
	}
	public double getHeadingDegrees() {
		return pigeon.getAngle();
	}
	public ChassisSpeeds getChassisSpeeds() {
		return kinematics.toChassisSpeeds(getModuleStates());
	}
	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];
		for (int i = 0; i < 4; i++) positions[i] = modules[i].getPosition();
		return positions;
	}
	public SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];
		for (int i = 0; i < 4; i++) states[i] = modules[i].getState();
		return states;
	}
	public void setEncoderOffsets() {
		Preferences.setDouble("FL offset", modules[0].findOffset());
		Preferences.setDouble("FR offset", modules[1].findOffset());
		Preferences.setDouble("BL offset", modules[2].findOffset());
		Preferences.setDouble("BR offset", modules[3].findOffset());
	}
	public Pose2d getPose() {
		return odometer.getEstimatedPosition();
	}
    public void resetOdometry(Pose2d pose) {
		zeroGyroscope(pose.getRotation().getDegrees());
        odometer.resetPosition(pose.getRotation(), getModulePositions(), pose);
    }
    public void resetOdometry() {
        odometer.resetPosition(getGyroscopeRotation(), getModulePositions(), DRIVE_ODOMETRY_ORIGIN);
    }
	/**
	 *  Sets the modules speed and rotation to zero.
	 */
	public void pointWheelsForward() {
		for (int i = 0; i < 4; i++) {
			setModule(i, new SwerveModuleState(0, new Rotation2d()));
		}
	}
	public void pointWheelsInward() {
		setModule(0, new SwerveModuleState(0, Rotation2d.fromDegrees(-135)));
		setModule(1, new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
		setModule(2, new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		setModule(3, new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
	}
	public void drive(ChassisSpeeds chassisSpeeds) {
		// SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(chassisSpeeds);
		SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(chassisSpeeds, 0.02)); //TODO see if this works as expected
		double maxSpeed = Collections.max(Arrays.asList(desiredStates)).speedMetersPerSecond;
		if (maxSpeed <= DriveConstants.DRIVE_DEADBAND_MPS) {
			for (int i = 0; i < 4; i++) {
				stop();
			}
		} else {
			setModuleStates(desiredStates);
		}
	}
	/**
	 * Sets all module drive speeds to 0, but leaves the wheel angles where they were.
	 */
	public void stop() {
		for (int i = 0; i < 4; i++) {
			modules[i].setDesiredState(new SwerveModuleState(0, lastAngles[i]));
		}
	}
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_VELOCITY_METERS_PER_SECOND);
		for (int i = 0; i < 4; i++) {
			setModule(i, desiredStates[i]);
		}
	}
	private void setModule(int i, SwerveModuleState desiredState) {
		modules[i].setDesiredState(desiredState);
		lastAngles[i] = desiredState.angle;
	}
	public void updateOdometry() {
		odometer.updateWithTime(Timer.getFPGATimestamp(), getGyroscopeRotation(), getModulePositions());
	}
	public void updateOdometryWithVision(Pose2d estematedPose, double timestampSeconds) {
		odometer.addVisionMeasurement(estematedPose, timestampSeconds);
	}
	public double calculateSpeakerAngle(){
		Pose2d dtvalues = this.getPose();
		// triangle for robot angle
		Optional<Alliance> alliance = DriverStation.getAlliance();
		if (alliance.isPresent() && alliance.get() == Alliance.Red) {
			deltaY = Math.abs(dtvalues.getY() - Field.SPEAKER_Y);
		} else {
			deltaY = Math.abs(dtvalues.getY() - Field.SPEAKER_Y);

		}
		if(DriverStation.getAlliance().get() == Alliance.Red) {
			deltaX = Math.abs(dtvalues.getX() - Field.BLUE_SPEAKER_X);
		} else {
			deltaX = Math.abs(dtvalues.getX() - Field.RED_SPEAKER_X);
		}
		speakerDist = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
		SmartDashboard.putNumber("dist to speakre", speakerDist);

		// RobotState.getInstance().ShooterInRange = speakerDist<Field.MAX_SHOOTING_DISTANCE;
		
		//getting desired robot angle
		// if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
			if (dtvalues.getY() >= Field.SPEAKER_Y) {
				//the robot is to the left of the speaker
				double thetaAbove = -Math.toDegrees(Math.asin(deltaX / speakerDist))-90;
				m_desiredRobotAngle = thetaAbove;
			}
			else{
				double thetaBelow = Math.toDegrees(Math.asin(deltaX / speakerDist))+90;
				m_desiredRobotAngle = thetaBelow;
			} 
		// } else {
		// if (dtvalues.getY() >= Field.RED_SPEAKER_Y) {
		// 	//the robot is to the left of the speaker
		// 	double thetaAbove = -Math.toDegrees(Math.asin(deltaX / speakerDist))-90;
		// 	m_desiredRobotAngle = thetaAbove;
		// }
		// else{
		// 	double thetaBelow = Math.toDegrees(Math.asin(deltaX / speakerDist))+90;
		// 	m_desiredRobotAngle = thetaBelow;
		// }
		// m_desiredRobotAngle = m_desiredRobotAngle + 180;
		// }
		SmartDashboard.putNumber("just angle", Math.toDegrees(Math.asin(deltaX / speakerDist)));
		SmartDashboard.putNumber("desired angle", m_desiredRobotAngle);
		return m_desiredRobotAngle;
	}
	
	public double calculateSpeakerAngleMoving(){
		dtvalues = this.getPose();
		Optional<Alliance> alliance = DriverStation.getAlliance();
		shootingSpeed = ShooterConstants.SHOOTING_SPEED_MPS;
		//triangle for robot angle
		if (alliance.isPresent() && alliance.get() == Alliance.Red) {
			deltaX = Math.abs(dtvalues.getX() - Field.RED_SPEAKER_X);
		} else {
			deltaX = Math.abs(dtvalues.getX() - Field.BLUE_SPEAKER_X);
		}
		deltaY = Math.abs(dtvalues.getY() - Field.SPEAKER_Y);
		speakerDist = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
		SmartDashboard.putNumber("dist to speakre", speakerDist);
		
		Rotation2d unadjustedAngle = Rotation2d.fromDegrees(Math.asin(deltaX/speakerDist));
		shootingTime = speakerDist/shootingSpeed; //calculates how long the note will take to reach the target
		currentXSpeed = this.getChassisSpeeds().vxMetersPerSecond;
		currentYSpeed = this.getChassisSpeeds().vyMetersPerSecond;
		targetOffset = new Translation2d(currentXSpeed*shootingTime*OFFSET_MULTIPLIER*unadjustedAngle.getRadians(), currentYSpeed*shootingTime*OFFSET_MULTIPLIER); 
		//line above calculates how much our current speed will affect the ending location of the note if it's in the air for ShootingTime
		
		//next 3 lines set where we actually want to aim, given the offset our shooting will have based on our speed
		int correctionDirection;
		double speakerX;
		if(DriverStation.getAlliance().get() == Alliance.Blue) {
			correctionDirection = 1;
			speakerX = Field.BLUE_SPEAKER_X;
		} else {
			correctionDirection = -1;
			speakerX = Field.RED_SPEAKER_X;
		}
		offsetSpeakerX = speakerX+(targetOffset.getX()*correctionDirection);
		offsetSpeakerY = Field.SPEAKER_Y+(targetOffset.getY()*correctionDirection);
		offsetDeltaX = Math.abs(dtvalues.getX() - offsetSpeakerX);
		offsetDeltaY = Math.abs(dtvalues.getY() - offsetSpeakerY);
		
		adjustedTarget = new Translation2d(offsetSpeakerX, offsetSpeakerY);
		offsetSpeakerdist = Math.sqrt(Math.pow(offsetDeltaY, 2) + Math.pow(offsetDeltaX, 2));
		SmartDashboard.putNumber("offsetSpeakerDis", offsetSpeakerdist);
		RobotState.getInstance().ShooterInRange = offsetSpeakerdist<Field.MAX_SHOOTING_DISTANCE;
		SmartDashboard.putString("offset amount", targetOffset.toString());
		SmartDashboard.putString("offset speaker location", new Translation2d(offsetSpeakerX, offsetSpeakerY).toString());
		//getting desired robot angle
		if (alliance.get() == Alliance.Blue) {
			if (dtvalues.getY() >= adjustedTarget.getY()) {
				double thetaAbove = -Math.toDegrees(Math.asin(offsetDeltaX / offsetSpeakerdist))-90;
				m_desiredRobotAngle = thetaAbove;
			}
			else{
				double thetaBelow = Math.toDegrees(Math.asin(offsetDeltaX / offsetSpeakerdist))+90;
				m_desiredRobotAngle = thetaBelow;
		} } else {
			if (dtvalues.getY() >= adjustedTarget.getY()) {
				double thetaAbove = Math.toDegrees(Math.asin(offsetDeltaX / offsetSpeakerdist))-90;
				m_desiredRobotAngle = thetaAbove;
			}
			else{
				double thetaBelow = -Math.toDegrees(Math.asin(offsetDeltaX / offsetSpeakerdist))+90;
				m_desiredRobotAngle = thetaBelow;
			}
		}
		MathRanNumber++;
		SmartDashboard.putString("adjusted target", adjustedTarget.toString());
		return m_desiredRobotAngle;
	}
	public double getSpeakerAngleDifference() {
		return calculateSpeakerAngleMoving()-(getGyroscopeRotation().getDegrees()%360);
	}
	public Pose2d getAverageBotPose(LimelightValues ll2, LimelightValues ll3) {
		double ll2X = ll2.getBotPoseBlue().getX();
		double ll3X = ll3.getBotPoseBlue().getX();
		double ll2Y = ll2.getBotPoseBlue().getY();
		double ll3Y = ll3.getBotPoseBlue().getY();
		double ll2rotation = ll2.getBotPoseBlue().getRotation().getRadians();
		double ll3rotation = ll3.getBotPoseBlue().getRotation().getRadians();
		Rotation2d averageRotation = new Rotation2d((ll2rotation+ll3rotation)/2);
		double averageX = (ll2X+ll3X)/2;
		double averageY = (ll2Y+ll3Y)/2;
		return new Pose2d(new Translation2d(averageX, averageY), averageRotation);
	}
	public void forceUpdateOdometryWithVision() {
		if(SmartDashboard.getBoolean("use limelight", false)) {
			if(SmartDashboard.getBoolean("use 2 limelights", false)) {
				LimelightValues ll2 = limelight.getLimelightValues(Vision.APRILTAG_LIMELIGHT2_NAME);
				LimelightValues ll3 = limelight.getLimelightValues(Vision.APRILTAG_LIMELIGHT3_NAME);
				if (ll2.isResultValid && !ll3.isResultValid) {updateOdometryWithVision(ll2.getBotPoseBlue(), ll2.gettimestamp());}
				if (!ll2.isResultValid && ll3.isResultValid) {updateOdometryWithVision(ll3.getBotPoseBlue(), ll3.gettimestamp());}
				if (ll2.isResultValid && ll3.isResultValid) {updateOdometryWithVision(getAverageBotPose(ll2, ll3), ll3.gettimestamp());}
			} else {
				LimelightValues ll2 = limelight.getLimelightValues(Vision.APRILTAG_LIMELIGHT2_NAME);
				if(ll2.isResultValid) {updateOdometryWithVision(ll2.getBotPoseBlue(), ll2.gettimestamp());}
			}
		}
	}
	@Override
	public void periodic() {
		SmartDashboard.putString("alliance:", DriverStation.getAlliance().get().toString());
		updateOdometry();
		AngleShooterSubsystem.setDTPose(getPose());
		AngleShooterSubsystem.setDTChassisSpeeds(getChassisSpeeds());
		if(SmartDashboard.getBoolean("force use limelight", false)) {
			forceUpdateOdometryWithVision();
		} else {
			if (SmartDashboard.getBoolean("use limelight", false)) {
				if(SmartDashboard.getBoolean("use 2 limelights", false)) {
					LimelightValues ll2 = limelight.getLimelightValues(Vision.APRILTAG_LIMELIGHT2_NAME);
					LimelightValues ll3 = limelight.getLimelightValues(Vision.APRILTAG_LIMELIGHT3_NAME);
					Boolean isLL2VisionValid = ll2.isResultValid;
					Boolean isLL3VisionValid = ll3.isResultValid;
					Boolean isLL2VisionTrustworthy = isLL2VisionValid && ll2.isPoseTrustworthy(odometer.getEstimatedPosition());
					Boolean isLL3VisionTrustworthy = isLL3VisionValid && ll3.isPoseTrustworthy(odometer.getEstimatedPosition());
					SmartDashboard.putBoolean("LL2visionValid", isLL2VisionTrustworthy);
					SmartDashboard.putBoolean("LL3visionValid", isLL3VisionTrustworthy);
					if (SmartDashboard.getBoolean("trust limelight", false)) {
						if (isLL2VisionTrustworthy && !isLL3VisionTrustworthy) {updateOdometryWithVision(ll2.getBotPoseBlue(), ll2.gettimestamp());}
						if (!isLL2VisionTrustworthy && isLL3VisionTrustworthy) {updateOdometryWithVision(ll3.getBotPoseBlue(), ll3.gettimestamp());}
						if (isLL2VisionTrustworthy && isLL3VisionTrustworthy) {updateOdometryWithVision(getAverageBotPose(ll2, ll3), ll3.gettimestamp());}
					}
				} else {
					LimelightValues ll2 = limelight.getLimelightValues(Vision.APRILTAG_LIMELIGHT2_NAME);
					Boolean isLL2VisionValid = ll2.isResultValid;
					Boolean isLL2VisionTrustworthy = isLL2VisionValid && ll2.isPoseTrustworthy(odometer.getEstimatedPosition());
					if (isLL2VisionTrustworthy) {
						updateOdometryWithVision(ll2.getBotPoseBlue(), ll2.gettimestamp());
					}
				}
			}
		}
		m_field.setRobotPose(odometer.getEstimatedPosition());
        SmartDashboard.putNumber("Robot Angle", getGyroscopeRotation().getDegrees());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
		//for testing RotateRobot:
		SmartDashboard.putNumber("loopedHeading", getHeadingLooped());
		SmartDashboard.putNumber("calculated speaker angle", calculateSpeakerAngle());
		SmartDashboard.putNumber("TESTING robot angle difference", getSpeakerAngleDifference());
	}
}
