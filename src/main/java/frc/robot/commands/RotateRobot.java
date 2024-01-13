package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.LimelightFiducialData;
import frc.robot.settings.LimelightValues;
import frc.robot.settings.Constants.Field;
import frc.robot.settings.Constants.ShooterConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;

public class RotateRobot extends Command {
    Limelight m_limelightClass;
    DrivetrainSubsystem m_drivetrain;
    LimelightFiducialData m_llvalues;
    double desiredRobotAngle;
    double currentHeading;
    double diffrenceAngle;
    double turningSpeed;
    
    public RotateRobot(Limelight limelightClass, DrivetrainSubsystem drivetrain, double currentHeading){
        m_limelightClass = limelightClass;
        m_drivetrain = drivetrain;
        this.currentHeading = currentHeading;
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d dtvalues = m_drivetrain.getPose();
    
    //triangle for robot angle
    double speakerA = Math.abs(dtvalues.getX() - Field.SPEAKER_X);
    double speakerB = Math.abs(dtvalues.getY() - Field.SPEAKER_Y);
    double speakerC = Math.sqrt(Math.pow(speakerA, 2) + Math.pow(speakerB, 2));

    //getting desired robot angle
    

    if (dtvalues.getY() <= Field.SPEAKER_Y) {
        double thetaBelow = (Math.asin(speakerA / speakerC)) + 90;
        desiredRobotAngle = thetaBelow;
    }
    else{
        double thetaAbove = 360 - (Math.asin(speakerA / speakerC) + 90);
        desiredRobotAngle = thetaAbove;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //move robot to desired angle

    diffrenceAngle = (desiredRobotAngle - this.currentHeading);
    SmartDashboard.putNumber("differenceAngle", diffrenceAngle);

    turningSpeed = diffrenceAngle * ShooterConstants.AUTO_AIM_kP;
    SmartDashboard.putNumber("turningSpeed", turningSpeed);

    m_drivetrain.drive(new ChassisSpeeds(0, 0, turningSpeed));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(diffrenceAngle) < ShooterConstants.SHOOTER_ANGLE_TOLERANCE;
  }
}
