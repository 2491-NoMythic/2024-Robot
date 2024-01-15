// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.settings.Constants.Field;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoAimParallel extends ParallelCommandGroup {
  DrivetrainSubsystem m_drivetrain;
  ShooterSubsystem m_ShooterSubsystem;
  double m_desiredRobotAngle;
  double differenceAngle;
  double currentHeading;
  double speakerDist;
  double speakerA;
  double speakerB;
  double m_DesiredShooterAngle;
  double turningSpeed;
  RotateRobot rotateRobot;
  AngleShooter angleShooter;
  int accumulativeTurns;
  /** Creates a new autoAimParallel. */
  public autoAimParallel( DrivetrainSubsystem drivetrain, ShooterSubsystem shooterSubsystem) {
    m_drivetrain = drivetrain;
    m_ShooterSubsystem = shooterSubsystem;

    addRequirements(drivetrain);
    addRequirements(shooterSubsystem);

    Pose2d dtvalues = m_drivetrain.getPose();
    currentHeading = m_drivetrain.getHeadingLooped();
    
    //triangle for robot angle
    speakerA = Math.abs(dtvalues.getX() - Field.SPEAKER_X);
    speakerB = Math.abs(dtvalues.getY() - Field.SPEAKER_Y);
    speakerDist = Math.sqrt(Math.pow(speakerA, 2) + Math.pow(speakerB, 2));

    //getting desired robot angle
    

    if (dtvalues.getY() <= Field.SPEAKER_Y) {
        double thetaBelow = (Math.asin(speakerA / speakerDist)) + 90;
        m_desiredRobotAngle = thetaBelow;
    }
    else{
        double thetaAbove = 360 - (Math.asin(speakerA / speakerDist) + 90);
        m_desiredRobotAngle = thetaAbove;
    }

    //triangle for shooter angle
    //TODO: Find out if we need projectile motion physics and implement it
    m_DesiredShooterAngle = Math.atan(Field.SPEAKER_Z / speakerDist);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RotateRobot(drivetrain, m_desiredRobotAngle, currentHeading),
      new AngleShooter(shooterSubsystem, m_DesiredShooterAngle)
    );
  }
}
