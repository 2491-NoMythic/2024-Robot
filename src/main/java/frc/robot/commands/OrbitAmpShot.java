// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.SequenceInputStream;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AngleShooterSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.*;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.ShooterConstants;

public class OrbitAmpShot extends SequentialCommandGroup {
  /** Creates a new OrbitAmpShot. */


  public OrbitAmpShot(AngleShooterSubsystem angleShooterSubsystem, IndexerSubsystem indexerSubsystem, ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(angleShooterSubsystem, indexerSubsystem,shooterSubsystem);

    addCommands(
      new InstantCommand(()->shooterSubsystem.shootRPS(40,20)),
      //working: 40, 20
      new ParallelRaceGroup(
      new AngleShooter(angleShooterSubsystem, ()-> 12),
      new WaitCommand(2
      )
      //working 2 second wait time
      ),
      new ParallelCommandGroup(
      
          new AngleShooter(angleShooterSubsystem, ()->ShooterConstants.MAXIMUM_SHOOTER_ANGLE),
          new SequentialCommandGroup(
           new WaitCommand(.2),
           new InstantCommand(()->indexerSubsystem.set(.3)),
           new WaitCommand(1)
         )
      )
    );


  }

  // Called when the command is initially scheduled.




}
