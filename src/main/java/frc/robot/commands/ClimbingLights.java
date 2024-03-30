// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights;

public class ClimbingLights extends Command {
  private Lights lights;
    // Store what the last brightness of the first pixel is
    private int loopFirstPixelValue;
  /** Creates a new ClimbingLights. */
  public ClimbingLights(Lights lights) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lights);
    this.lights = lights;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    for (int i = 0; i < lights.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final int brightness = (loopFirstPixelValue + (i * 180 / lights.getLength())) % 180;
      // Set the value
      lights.setLightsHSV(0, i,  300, 255, brightness);
    }
    // Increase by to make the rainbow "move"
    loopFirstPixelValue += 3;
    // Check bounds
    loopFirstPixelValue %= 180;
    lights.dataSetter();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lights.lightsOut();
    lights.dataSetter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
