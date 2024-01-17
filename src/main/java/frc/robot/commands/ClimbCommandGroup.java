// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.settings.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbCommandGroup extends SequentialCommandGroup {
  public Climber climber; 
  public double speed;
  /** Creates a new ClimberClimb. */
  public ClimbCommandGroup( Climber climber, double speed) {
    // Add your commands in the addCommands() call, e.g.
    addRequirements(climber);
    this.climber = climber;
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new RunClimberUp(speed, climber), new WaitCommand(1), new RunClimberDown(speed, climber));
  }
}
