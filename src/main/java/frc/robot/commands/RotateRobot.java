package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.Field;
import frc.robot.settings.Constants.ShooterConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import static frc.robot.settings.Constants.DriveConstants.*;
import static frc.robot.settings.Constants.ShooterConstants.AUTO_AIM_ROBOT_kD;
import static frc.robot.settings.Constants.ShooterConstants.AUTO_AIM_ROBOT_kI;
import static frc.robot.settings.Constants.ShooterConstants.AUTO_AIM_ROBOT_kP;
import static frc.robot.settings.Constants.ShooterConstants.AUTO_AIM_SHOOTER_kP;

public class RotateRobot extends Command {
    DrivetrainSubsystem m_drivetrain;
    double desiredRobotAngle;
    double currentHeading;
    double differenceAngle;
    double turningSpeed;
    PIDController speedController;
    
  public RotateRobot(DrivetrainSubsystem drivetrain, double desiredRobotAngle){
        m_drivetrain = drivetrain;
        this.desiredRobotAngle = desiredRobotAngle;
        speedController = new PIDController(
          AUTO_AIM_ROBOT_kP, 
          AUTO_AIM_ROBOT_kI,
          AUTO_AIM_ROBOT_kD);
          speedController.setTolerance(ShooterConstants.ROBOT_ANGLE_TOLERANCE);
          speedController.enableContinuousInput(-180, 180);
          addRequirements(drivetrain);
        }
        
        // Called when the command is initially scheduled.
        @Override
        public void initialize() {
          speedController.setSetpoint(0);
      }
      
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
        //move robot to desired angle
        this.currentHeading = m_drivetrain.getHeadingDegrees();

        differenceAngle = (desiredRobotAngle - this.currentHeading);
        m_drivetrain.drive(new ChassisSpeeds(0, 0, speedController.calculate(differenceAngle)));

        SmartDashboard.putNumber("current Heading", m_drivetrain.getHeadingDegrees());
        SmartDashboard.putNumber("difference", differenceAngle);
        SmartDashboard.putNumber("PID calculated output", speedController.calculate(differenceAngle));
    
      }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(differenceAngle) < ShooterConstants.ROBOT_ANGLE_TOLERANCE;
  }
}
