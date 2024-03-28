// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.settings.Constants.IndexerConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
This command will cancel any default commands for these subsytems
 **/
public class OverrideCommand extends Command {
  public OverrideCommand(Subsystem... subsystems) {
    addRequirements(subsystems);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
