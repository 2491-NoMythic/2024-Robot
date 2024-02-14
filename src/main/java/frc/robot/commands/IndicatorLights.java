// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.RobotState;
public class IndicatorLights extends Command {
 
  Lights lights;

  public IndicatorLights(Lights lights) {
    addRequirements(lights);
    this.lights = lights;
  }


  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (RobotState.getInstance().IsNoteSeen) {
    lights.setSectionOne(50,40,0);
    } else {
      lights.setSectionOne(50, 0, 50);
    }
    if (RobotState.getInstance().ShooterInRange && RobotState.getInstance().ShooterReady){
      lights.setSectionTwo(0,50,0);
    } else if (RobotState.getInstance().ShooterInRange){
      lights.setSectionTwo(50, 50, 0);
    } else {
      lights.setSectionTwo(50, 0, 50);
    }
    lights.dataSetter();
  }


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
