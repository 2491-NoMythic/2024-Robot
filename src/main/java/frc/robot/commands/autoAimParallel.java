// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.settings.Constants.Field;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import static frc.robot.settings.Constants.ShooterConstants.*;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem; 

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoAimParallel extends ParallelCommandGroup {
  DrivetrainSubsystem m_drivetrain;
  ShooterSubsystem m_ShooterSubsystem;

  /** Creates a new autoAimParallel. */
  public autoAimParallel( DrivetrainSubsystem drivetrain /*, ShooterSubsystem shooter */) {
    m_drivetrain = drivetrain;
    // m_ShooterSubsystem = shooterSubsystem;

    addRequirements(drivetrain);
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RotateRobot(drivetrain, drivetrain::calculateSpeakerAngle)//,
      // new AngleShooter(shooterSubsystem, shooterSubsystem.calculateSpeakerAngle(drivetrain.getPose()))
    );
  }
}
