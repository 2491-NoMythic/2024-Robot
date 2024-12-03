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
import static frc.robot.settings.Constants.Vision.APRILTAG_LIMELIGHT2_NAME;
import static frc.robot.settings.Constants.Vision.APRILTAG_LIMELIGHT3_NAME;

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
import frc.robot.commands.RotateRobot;
import frc.robot.helpers.MotorLogger;
import frc.robot.helpers.MythicalMath;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.CTREConfigs;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.settings.Constants.Field;

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

	Limelight limelight;
	MotorLogger[] motorLoggers;
	PIDController speedController;

	public DrivetrainSubsystem() {
		this.limelight=Limelight.getInstance();
		Preferences.initDouble("FL offset", 0);
		Preferences.initDouble("FR offset", 0);
		Preferences.initDouble("BL offset", 0);
		Preferences.initDouble("BR offset", 0);
		PathPlannerLogging.setLogActivePathCallback((poses) -> m_field.getObject("path").setPoses(poses));
		SmartDashboard.putData("Field", m_field);
		SmartDashboard.putBoolean("Vision/force use limelight", false);

		//creates and configures each of the four swerve modules used in the drivetrain
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
		//configures the odometer
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
		SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(chassisSpeeds, 0.02));
		double maxSpeed = Collections.max(Arrays.asList(desiredStates)).speedMetersPerSecond;
		if (maxSpeed <= DriveConstants.DRIVE_DEADBAND_MPS) {
			for (int i = 0; i < 4; i++) {
				stop();
			}
		} else {
			setModuleStates(desiredStates);
		}
	}

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
		LimelightHelpers.SetRobotOrientation(APRILTAG_LIMELIGHT2_NAME, odometer.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
		LimelightHelpers.SetRobotOrientation(APRILTAG_LIMELIGHT3_NAME, odometer.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);

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

	@Override
	public void periodic() {
		updateOdometry();
		//sets the robot orientation for each of the limelights, which is required for the
		if (Preferences.getBoolean("Use Limelight", false)) {
			if (SmartDashboard.getBoolean("Vision/force use limelight", false)) {
				forceUpdateOdometryWithVision();
			} else {
				updateOdometryWithVision();
			}
		} else {
			RobotState.getInstance().LimelightsUpdated = false;
		}
	
		m_field.setRobotPose(odometer.getEstimatedPosition());
		RobotState.getInstance().odometerOrientation = getOdometryRotation().getDegrees();
		//updates logging for all drive motors on the swerve modules
		for (int i = 0; i < 4; i++) {
			motorLoggers[i].log(modules[i].getDriveMotor());
		}
		//puts important data for testing features on SmartDashboard
		SmartDashboard.putNumber("Robot Angle", getOdometryRotation().getDegrees());
		SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
		SmartDashboard.putNumber("DRIVETRAIN/forward speed", getChassisSpeeds().vxMetersPerSecond);
		SmartDashboard.putNumber("DRIVETRAIN/rotational speed", Math.toDegrees(getChassisSpeeds().omegaRadiansPerSecond));
		SmartDashboard.putNumber("DRIVETRAIN/gyroscope rotation degrees", getPose().getRotation().getDegrees());
		SmartDashboard.putNumber("DRIVETRAIN/degrees per second", Math.toDegrees(getChassisSpeeds().omegaRadiansPerSecond));

		Logger.recordOutput("MyStates", getModuleStates());
		Logger.recordOutput("Position", odometer.getEstimatedPosition());
		Logger.recordOutput("Gyro", getGyroscopeRotation());
	}
}
