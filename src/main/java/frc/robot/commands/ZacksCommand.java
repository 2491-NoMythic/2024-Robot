// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveMeters;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.commands.RotateRobot;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZacksCommand extends SequentialCommandGroup {
DrivetrainSubsystem drivetrain;
  /** Creates a new ZacksCommand. */
  public ZacksCommand(DrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveMeters(drivetrain, 2.8, 1, 0, 0),
      new RotateRobot(drivetrain, ()->-90),
      new MoveMeters(drivetrain, 1.2, 1, 0, 0),
      new RotateRobot(drivetrain, ()->80),
      new MoveMeters(drivetrain, 1.3, 1, 0, 0)
      
      
      
      
    );
  }
}
