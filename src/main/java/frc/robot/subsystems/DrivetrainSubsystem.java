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
import static frc.robot.settings.Constants.DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;
import static frc.robot.settings.Constants.ShooterConstants.AUTO_AIM_ROBOT_kD;
import static frc.robot.settings.Constants.ShooterConstants.AUTO_AIM_ROBOT_kI;
import static frc.robot.settings.Constants.ShooterConstants.AUTO_AIM_ROBOT_kP;
import static frc.robot.settings.Constants.ShooterConstants.OFFSET_MULTIPLIER;
import static frc.robot.settings.Constants.Vision.APRILTAG_LIMELIGHT2_NAME;
import static frc.robot.settings.Constants.Vision.APRILTAG_LIMELIGHT3_NAME;
import static frc.robot.settings.Constants.Vision.OBJ_DETECTION_LIMELIGHT_NAME;

import java.util.Arrays;
import java.util.Collections;
import java.util.Optional;
import java.util.function.DoubleSupplier;
//import java.util.logging.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.google.flatbuffers.DoubleVector;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.commands.AngleShooter;
import frc.robot.commands.RotateRobot;
import frc.robot.helpers.MotorLogger;
import frc.robot.helpers.MythicalMath;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.CTREConfigs;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.settings.Constants.Field;
import frc.robot.settings.Constants.ShooterConstants;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;


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
	int runsValid;
	double MathRanNumber;
	MotorLogger[] motorLoggers;
	PIDController speedController;

	public DrivetrainSubsystem() {
		SmartDashboard.putNumber("CALLIBRATION/redRobotX", Field.CALCULATED_SHOOTER_RED_SPEAKER_X);
		SmartDashboard.putNumber("CALLIBRATION/blueRobotX", Field.CALCULATED_SHOOTER_BLUE_SPEAKER_X);
		SmartDashboard.putNumber("CALLIBRATION/blueY", Field.CALCULATED_BLUE_SPEAKER_Y);
		SmartDashboard.putNumber("CALLIBRATION/redY", Field.CALCULATED_RED_SPEAKER_Y);

		speedController = new PIDController(
			AUTO_AIM_ROBOT_kP, 
			AUTO_AIM_ROBOT_kI,
			AUTO_AIM_ROBOT_kD);
		speedController.setTolerance(ShooterConstants.ROBOT_ANGLE_TOLERANCE);
		speedController.enableContinuousInput(-180, 180);

		MathRanNumber = 0;
		runsValid = 0;
		this.limelight=Limelight.getInstance();

		Preferences.initDouble("FL offset", 0);
		Preferences.initDouble("FR offset", 0);
		Preferences.initDouble("BL offset", 0);
		Preferences.initDouble("BR offset", 0);
		PathPlannerLogging.setLogActivePathCallback((poses) -> m_field.getObject("path").setPoses(poses));
		SmartDashboard.putData("Field", m_field);
		SmartDashboard.putBoolean("Vision/force use limelight", false);
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

		DataLog log = DataLogManager.getLog();
		motorLoggers = new MotorLogger[] {
			new MotorLogger(log, "/drivetrain/motorFL"),
			new MotorLogger(log, "/drivetrain/motorFR"),
			new MotorLogger(log, "/drivetrain/motorBL"),
			new MotorLogger(log, "/drivetrain/motorBR"),
		};
		
		odometer = new SwerveDrivePoseEstimator(
			kinematics, 
			getGyroscopeRotation(),
			getModulePositions(),
			DRIVE_ODOMETRY_ORIGIN);
		odometer.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 99999999));
		}
	/**
	 * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
	 * 'forwards' direction.
	 */
	public void zeroGyroscope() {
		if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
			zeroGyroscope(180);
		} else {
			zeroGyroscope(0);
		}
	}
	public void zeroGyroscope(double angleDeg) {
		resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(angleDeg)));
	}
	public double getHeadingLooped() {
		//returns the heading of the robot, but only out of 360, not accumulative
		accumulativeLoops = (int) (getHeadingDegrees()/180); //finding the amount of times that 360 goes into the heading, as an int
		return getHeadingDegrees()-180*(accumulativeLoops); 
	}
	public Rotation2d getGyroscopeRotation() {
		return pigeon.getRotation2d();
	}
	/**
	 * 
	 * @return a rotation2D of the angle according to the odometer
	 */
	public Rotation2d getOdometryRotation() {
		return odometer.getEstimatedPosition().getRotation();
	}
	public double getHeadingDegrees() {
		return odometer.getEstimatedPosition().getRotation().getDegrees();
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
		odometer.resetPosition(getGyroscopeRotation(), getModulePositions(), pose);
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

	public void driveWhileAimed(ChassisSpeeds chassisSpeeds) {
		ChassisSpeeds desiredSpeeds = new ChassisSpeeds(
			chassisSpeeds.vxMetersPerSecond,
			chassisSpeeds.vyMetersPerSecond,
			speedController.calculate(getOdometryRotation().getDegrees()));
		drive(desiredSpeeds);
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
	/**
	 * Provide the odometry a vision pose estimate, only if there is a trustworthy pose available.
	 * <p>
	 * Each time a vision pose is supplied, the odometry pose estimation will change a little, 
	 * larger pose shifts will take multiple calls to complete.
	 */
	public void updateOdometryWithVision() {
		PoseEstimate estimate = limelight.getTrustedPose();
		if (estimate != null) {
			boolean doRejectUpdate = false;
			if(Math.abs(pigeon.getRate()) > 720) {
				doRejectUpdate = true;
			} 
			if(estimate.tagCount == 0) {
				doRejectUpdate = true;
			}
			if(!doRejectUpdate) {
				Logger.recordOutput("Vision/MergesPose", estimate.pose);
				odometer.addVisionMeasurement(estimate.pose, estimate.timestampSeconds);
			}
			RobotState.getInstance().LimelightsUpdated = true;
		} else {
			RobotState.getInstance().LimelightsUpdated = false;
		}
	}
	/**
	 * Set the odometry using the current apriltag estimate, disregarding the pose trustworthyness.
	 * <p>
	 * You only need to run this once for it to take effect.
	 */
	public void forceUpdateOdometryWithVision() {
		PoseEstimate estimate = limelight.getTrustedPose();
		if (estimate != null) {
			resetOdometry(estimate.pose);
		} else {
			System.err.println("No valid limelight estimate to reset from. (Drivetrain.forceUpdateOdometryWithVision)");
		}
	}
/**
 * DISCONTINUED
 * <p>
 * use calculateSpeakerAngleMoving() instead
 * @return claculated robot angle for the speaker, but bad
 */
	public double calculateSpeakerAngle(){
		Pose2d dtvalues = this.getPose();
		// triangle for robot angle
		Optional<Alliance> alliance = DriverStation.getAlliance();
		if (alliance.isPresent() && alliance.get() == Alliance.Red) {
			deltaY = Math.abs(dtvalues.getY() - Field.RED_SPEAKER_Y);
		} else {
			deltaY = Math.abs(dtvalues.getY() - Field.BLUE_SPEAKER_Y);

		}
		if(alliance.isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
			deltaX = Math.abs(dtvalues.getX() - Field.ROBOT_BLUE_SPEAKER_X);
		} else {
			deltaX = Math.abs(dtvalues.getX() - Field.ROBOT_RED_SPEAKER_X);
		}
		speakerDist = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
		// SmartDashboard.putNumber("dist to speaker", speakerDist);

		// RobotState.getInstance().ShooterInRange = speakerDist<Field.MAX_SHOOTING_DISTANCE;
		
		//getting desired robot angle
		// if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
			if (dtvalues.getY() >= Field.RED_SPEAKER_Y) {
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
		double speakerY;
		shootingSpeed = ShooterConstants.SHOOTING_SPEED_MPS;
		//triangle for robot angle
		if (alliance.isPresent() && alliance.get() == Alliance.Red) {
			deltaX = Math.abs(dtvalues.getX() - Field.ROBOT_RED_SPEAKER_X);
			speakerY = Field.RED_SPEAKER_Y;
		} else {
			deltaX = Math.abs(dtvalues.getX() - Field.ROBOT_BLUE_SPEAKER_X);
			speakerY = Field.BLUE_SPEAKER_Y;
		}
		deltaY = Math.abs(dtvalues.getY() - speakerY);
		speakerDist = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
		// SmartDashboard.putNumber("dist to speaker", speakerDist);
		
		Rotation2d unadjustedAngle = Rotation2d.fromRadians(Math.asin(deltaX/speakerDist));
		double totalDistToSpeaker = Math.sqrt(Math.pow(Field.SPEAKER_Z-ShooterConstants.SHOOTER_HEIGHT, 2) + Math.pow(speakerDist, 2));
		shootingTime = totalDistToSpeaker/shootingSpeed; //calculates how long the note will take to reach the target
		currentXSpeed = this.getChassisSpeeds().vxMetersPerSecond;
		currentYSpeed = this.getChassisSpeeds().vyMetersPerSecond;
		targetOffset = new Translation2d(currentXSpeed*shootingTime*OFFSET_MULTIPLIER*unadjustedAngle.getRadians(), currentYSpeed*shootingTime*OFFSET_MULTIPLIER); 
		//line above calculates how much our current speed will affect the ending location of the note if it's in the air for ShootingTime
		
		//next 3 lines set where we actually want to aim, given the offset our shooting will have based on our speed
		int correctionDirection;
		double speakerX;
		if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
			correctionDirection = 1;
			speakerX = Field.ROBOT_BLUE_SPEAKER_X;
		} else {
			correctionDirection = -1;
			speakerX = Field.ROBOT_RED_SPEAKER_X;
		}
		offsetSpeakerX = speakerX+(targetOffset.getX()*correctionDirection);
		offsetSpeakerY = speakerY+(targetOffset.getY()*correctionDirection);
		offsetDeltaX = Math.abs(dtvalues.getX() - offsetSpeakerX);
		offsetDeltaY = Math.abs(dtvalues.getY() - offsetSpeakerY);
		
		adjustedTarget = new Translation2d(offsetSpeakerX, offsetSpeakerY);
		offsetSpeakerdist = Math.sqrt(Math.pow(offsetDeltaY, 2) + Math.pow(offsetDeltaX, 2));
		SmartDashboard.putNumber("MATH/Robot speaker dist", offsetSpeakerdist);
		RobotState.getInstance().ShooterInRange = offsetSpeakerdist<Field.MAX_SHOOTING_DISTANCE;
		// SmartDashboard.putString("offset amount", targetOffset.toString());
		// SmartDashboard.putString("offset speaker location", new Translation2d(offsetSpeakerX, offsetSpeakerY).toString());
		//getting desired robot angle
		if (DriverStation.getAlliance().isPresent() && alliance.get() == Alliance.Blue) {
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

	public double calculateSpeakerAngleMovingWithSuppliers(DoubleSupplier redYSup, DoubleSupplier blueYSup, DoubleSupplier redRobotXSup, DoubleSupplier blueRobotXSup){
		double redY = redYSup.getAsDouble();
		double blueY = blueYSup.getAsDouble();
		double blueRobotX = blueRobotXSup.getAsDouble();
		double redRobotX = redRobotXSup.getAsDouble();
		dtvalues = this.getPose();
		Optional<Alliance> alliance = DriverStation.getAlliance();
		double speakerY;
		shootingSpeed = ShooterConstants.SHOOTING_SPEED_MPS;
		//triangle for robot angle
		if (alliance.isPresent() && alliance.get() == Alliance.Red) {
			deltaX = Math.abs(dtvalues.getX() - redRobotX);
			speakerY = redY;
		} else {
			deltaX = Math.abs(dtvalues.getX() - blueRobotX);
			speakerY = blueY;
		}
		deltaY = Math.abs(dtvalues.getY() - speakerY);
		speakerDist = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
		// SmartDashboard.putNumber("dist to speaker", speakerDist);
		
		Rotation2d unadjustedAngle = Rotation2d.fromRadians(Math.asin(deltaX/speakerDist));
		double totalDistToSpeaker = Math.sqrt(Math.pow(Field.SPEAKER_Z-ShooterConstants.SHOOTER_HEIGHT, 2) + Math.pow(speakerDist, 2));
		shootingTime = totalDistToSpeaker/shootingSpeed; //calculates how long the note will take to reach the target
		currentXSpeed = this.getChassisSpeeds().vxMetersPerSecond;
		currentYSpeed = this.getChassisSpeeds().vyMetersPerSecond;
		targetOffset = new Translation2d(currentXSpeed*shootingTime*OFFSET_MULTIPLIER*unadjustedAngle.getRadians(), currentYSpeed*shootingTime*OFFSET_MULTIPLIER); 
		//line above calculates how much our current speed will affect the ending location of the note if it's in the air for ShootingTime
		
		//next 3 lines set where we actually want to aim, given the offset our shooting will have based on our speed
		int correctionDirection;
		double speakerX;
		if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
			correctionDirection = 1;
			speakerX = blueRobotX;
		} else {
			correctionDirection = -1;
			speakerX = redRobotX;
		}
		offsetSpeakerX = speakerX+(targetOffset.getX()*correctionDirection);
		offsetSpeakerY = speakerY+(targetOffset.getY()*correctionDirection);
		offsetDeltaX = Math.abs(dtvalues.getX() - offsetSpeakerX);
		offsetDeltaY = Math.abs(dtvalues.getY() - offsetSpeakerY);
		
		adjustedTarget = new Translation2d(offsetSpeakerX, offsetSpeakerY);
		offsetSpeakerdist = Math.sqrt(Math.pow(offsetDeltaY, 2) + Math.pow(offsetDeltaX, 2));
		SmartDashboard.putNumber("MATH/Robot speaker dist", offsetSpeakerdist);
		RobotState.getInstance().ShooterInRange = offsetSpeakerdist<Field.MAX_SHOOTING_DISTANCE;
		// SmartDashboard.putString("offset amount", targetOffset.toString());
		// SmartDashboard.putString("offset speaker location", new Translation2d(offsetSpeakerX, offsetSpeakerY).toString());
		//getting desired robot angle
		if (DriverStation.getAlliance().isPresent() && alliance.get() == Alliance.Blue) {
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
	private double getSpeakerAngleDifference() {
		return calculateSpeakerAngleMoving()-(getOdometryRotation().getDegrees()%360);
	}
	public boolean validShot() {
		return runsValid >= Constants.LOOPS_VALID_FOR_SHOT;
	}


	@Override
	public void periodic() {
		if (DriverStation.getAlliance().isPresent()) {
			SmartDashboard.putString("alliance:", DriverStation.getAlliance().get().toString());
		}
		speedController.setSetpoint(calculateSpeakerAngleMoving());
		updateOdometry();
		AngleShooterSubsystem.setDTPose(getPose());
		AngleShooterSubsystem.setDTChassisSpeeds(getChassisSpeeds());
		if (Preferences.getBoolean("Use Limelight", false)) {
			LimelightHelpers.SetRobotOrientation(APRILTAG_LIMELIGHT2_NAME, odometer.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
			LimelightHelpers.SetRobotOrientation(APRILTAG_LIMELIGHT3_NAME, odometer.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
			if (SmartDashboard.getBoolean("Vision/force use limelight", false)) {
				forceUpdateOdometryWithVision();
			} else {
				updateOdometryWithVision();
			}
		} else {
			RobotState.getInstance().LimelightsUpdated = false;
		}
	
		m_field.setRobotPose(odometer.getEstimatedPosition());
        SmartDashboard.putNumber("Robot Angle", getOdometryRotation().getDegrees());
		RobotState.getInstance().odometerOrientation = getOdometryRotation().getDegrees();
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
		SmartDashboard.putNumber("calculated speaker angle", calculateSpeakerAngleMoving());
		SmartDashboard.putNumber("TESTING robot angle difference", getSpeakerAngleDifference());
		if (getSpeakerAngleDifference()<DriveConstants.ALLOWED_ERROR) {
			runsValid++;
		} else {
			runsValid = 0;
		}
		for (int i = 0; i < 4; i++) {
			motorLoggers[i].log(modules[i].getDriveMotor());
		}
		SmartDashboard.putNumber("DRIVETRAIN/forward speed", getChassisSpeeds().vxMetersPerSecond);
		SmartDashboard.putNumber("DRIVETRAIN/rotational speed", Math.toDegrees(getChassisSpeeds().omegaRadiansPerSecond));
		SmartDashboard.putNumber("DRIVETRAIN/gyroscope rotation degrees", getPose().getRotation().getDegrees());
		SmartDashboard.putNumber("DRIVETRAIN/degrees per second", Math.toDegrees(getChassisSpeeds().omegaRadiansPerSecond));

		SmartDashboard.putNumber("CALLIBRATION/Supplied calculated robot angle", calculateSpeakerAngleMovingWithSuppliers(()->SmartDashboard.getNumber("CALLIBRATION/redY", 0), ()->SmartDashboard.getNumber("CALLIBRATION/blueY", 0), ()->SmartDashboard.getNumber("CALLIBRATION/redRobotX", 0), ()->SmartDashboard.getNumber("CALLIBRATION/blueRobotX", 0)));
		for (int i = 0; i < 4; i++) {
			motorLoggers[i].log(modules[i].getDriveMotor());
		}

		Logger.recordOutput("MyStates", getModuleStates());
		Logger.recordOutput("Position", odometer.getEstimatedPosition());
		Logger.recordOutput("Gyro", getGyroscopeRotation());
		
		//Logger.recordOutput("Vision/targetposes/NotePoses/FieldSpace", robotToFieldCoordinates(LimelightHelpers.getTargetPose3d_RobotSpace(OBJ_DETECTION_LIMELIGHT_NAME)));
		//liamsEstimates();
		
	}

	/**
	 * 
	 * @param robotCoordinates - put in the coordinates that have an origin of the robot
	 * @return
	 */
	private Pose3d robotToFieldCoordinates(Pose3d robotCoordinates)
	{
		double robotCoordinatesX = robotCoordinates.getX();
		double robotCoordinatesY = robotCoordinates.getY();
		double robotCoordinatesZ = robotCoordinates.getZ();
		Rotation3d robotCoordinatesAngle = robotCoordinates.getRotation();

		double odometrPoseX = odometer.getEstimatedPosition().getX();
		double odometrPosey = odometer.getEstimatedPosition().getY();


		return new Pose3d(robotCoordinatesX+odometrPoseX,robotCoordinatesY+odometrPosey,robotCoordinatesZ, robotCoordinatesAngle);
	}
}
