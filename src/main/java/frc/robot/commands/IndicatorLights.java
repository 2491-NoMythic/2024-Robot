// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.RobotState;

/*
 * 
 */
public class IndicatorLights extends Command {
 
  Lights lights;
 /**
 * This command controls the lights.  Section One Lights: If a note is seen, lights = dark orange-brown. Otherwise,
 *  lights = oddly deep dark purple.  Section Two Lights: If the shooter is in range, and is ready, lights = dark
 * green. If it is in range, but not ready, lights = a dark green-yellow-brown. If neither is true, lights = deep dark purple
 */
public IndicatorLights(Lights lights) {
    addRequirements(lights);
    this.lights = lights;
  }


  @Override
  public void initialize() {}

  @Override
  public void execute() {
    boolean noteInRobot = RobotState.getInstance().IsNoteHeld;
    boolean noteSeenByLimelight = RobotState.getInstance().IsNoteSeen;
    boolean limelightsUpdated = RobotState.getInstance().LimelightsUpdated;
    boolean readyToShoot = noteInRobot&&limelightsUpdated;
    boolean allSystemsGoodToGo = RobotState.getInstance().ShooterReady;
    if(readyToShoot) {
      if(allSystemsGoodToGo) {
        lights.setShooterFaceLights(50, 50, 50);
        lights.setShooterRailLights(0, 50, 0);
        lights.setOtherLights(0, 50, 0);
      } else {
        lights.setShooterFaceLights(0, 50, 0);
        lights.setShooterRailLights(0, 50, 0);
        lights.setOtherLights(0, 50, 0);
      }
    }
    if(!noteInRobot&&noteSeenByLimelight) {
      lights.setShooterFaceLights(50, 25, 25);
      lights.setShooterRailLights(50, 25, 25);
      lights.setOtherLights(50, 25, 25);
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
