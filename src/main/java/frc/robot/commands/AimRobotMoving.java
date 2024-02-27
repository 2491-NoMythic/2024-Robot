package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.settings.Constants.ShooterConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import static frc.robot.settings.Constants.ShooterConstants.AUTO_AIM_ROBOT_kD;
import static frc.robot.settings.Constants.ShooterConstants.AUTO_AIM_ROBOT_kI;
import static frc.robot.settings.Constants.ShooterConstants.AUTO_AIM_ROBOT_kP;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class AimRobotMoving extends Command {
    DrivetrainSubsystem m_drivetrain;
    DoubleSupplier desiredRobotAngleSupplier;
    double desiredRobotAngle;
    double currentHeading;
    double differenceAngle;
    double turningSpeed;
    PIDController speedController;
    DoubleSupplier translationXSupplier;
    DoubleSupplier translationYSupplier;
    BooleanSupplier run;
    BooleanSupplier cancel;
    DoubleSupplier rotationSupplier;
    double rotationSpeed;
    double allianceOffset;
    
  public AimRobotMoving(DrivetrainSubsystem drivetrain, DoubleSupplier rotationSupplier, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, BooleanSupplier run, BooleanSupplier cancel){
        m_drivetrain = drivetrain;
        speedController = new PIDController(
          AUTO_AIM_ROBOT_kP, 
          AUTO_AIM_ROBOT_kI,
          AUTO_AIM_ROBOT_kD);
          speedController.setTolerance(ShooterConstants.ROBOT_ANGLE_TOLERANCE);
          speedController.enableContinuousInput(-180, 180);
          this.translationXSupplier = translationXSupplier;
          this.translationYSupplier = translationYSupplier;
          this.rotationSupplier = rotationSupplier;
          this.cancel = cancel;
          this.run = run;
          addRequirements(drivetrain);
        }
        
        // Called when the command is initially scheduled.
        @Override
        public void initialize() {
          SmartDashboard.putBoolean("isRotateRunning", true);
          
        }
        
        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
          desiredRobotAngle = m_drivetrain.calculateSpeakerAngleMoving();
          speedController.setSetpoint(desiredRobotAngle);
        //move robot to desired angle
        this.currentHeading = m_drivetrain.getGyroscopeRotation().getDegrees();
        if(Math.abs(rotationSupplier.getAsDouble()) > 0.3) {
          rotationSpeed = rotationSupplier.getAsDouble() * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        } else {
          rotationSpeed = speedController.calculate(currentHeading);
        }
        if(DriverStation.getAlliance().get() == Alliance.Red) {
          allianceOffset = Math.PI;
        } else {
          allianceOffset = 0;
        }
        if(!cancel.getAsBoolean()) {
          m_drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
            translationXSupplier.getAsDouble() * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
            translationYSupplier.getAsDouble() * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
            rotationSpeed,
            new Rotation2d(m_drivetrain.getGyroscopeRotation().getRadians()+allianceOffset)));
        }
        // m_drivetrain.drive(new ChassisSpeeds(
        //   translationXSupplier.getAsDouble() * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
        //   translationYSupplier.getAsDouble() * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
        //   speedController.calculate(differenceAngle)));

        SmartDashboard.putNumber("current Heading", m_drivetrain.getGyroscopeRotation().getDegrees()%360);
        SmartDashboard.putNumber("difference", differenceAngle);
        SmartDashboard.putNumber("desired angle", desiredRobotAngle);
        SmartDashboard.putNumber("PID calculated output", speedController.calculate(differenceAngle));
    
      }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
    SmartDashboard.putBoolean("isRotateRunning", false);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!run.getAsBoolean() || cancel.getAsBoolean());
  }
}
