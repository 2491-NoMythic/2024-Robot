// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.NamedCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.AngleShooter;
import frc.robot.commands.GroundIntake;
import frc.robot.settings.Constants.ShooterConstants;
import frc.robot.subsystems.AngleShooterSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EntireGroundIntake extends ParallelCommandGroup {
IntakeSubsystem m_Intake;
IndexerSubsystem m_Indexer;
AngleShooterSubsystem m_Angler;
GroundIntake groundIntake = new GroundIntake(m_Intake, m_Indexer);

  /** Creates a new EntireGroundIntake. */
  public EntireGroundIntake() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(groundIntake, new InstantCommand(()-> m_Angler.setDesiredShooterAngle(ShooterConstants.GROUND_INTAKE_SHOOTER_ANGLE), m_Angler));
  }
}
