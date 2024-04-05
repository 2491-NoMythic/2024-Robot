// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.RobotState;

/*
 * 
 */
public class IndicatorLights extends Command {
 
  Lights lights;
  Timer timer;
  boolean noteWasIn;
 /**
 * This command controls the lights.  Middle Lights: if limelights are updating the odometry, these lights are green, otherwise they are red.
 * Side Lights: If a note is held, they are red if the shooter isn't rev'ed up, and green if they are rev'ed up. If a note is not held, they are off unless a note is seen by the intake
 * limelight, in which case they are yellow
 */
public IndicatorLights(Lights lights) {
    addRequirements(lights);
    this.lights = lights;
    timer = new Timer();
    timer.start();
  }


  @Override
  public void initialize() {}

  @Override
  public void execute() {
    boolean noteInRobot = RobotState.getInstance().IsNoteHeld;
    boolean noteSeenByLimelight = RobotState.getInstance().IsNoteSeen;
    boolean limelightsUpdated = RobotState.getInstance().LimelightsUpdated;
    boolean readyToShoot = noteInRobot&&limelightsUpdated;

    if(limelightsUpdated) {
      lights.setMid(0, 40, 0);
    } else {
      lights.setMid(50, 0, 0);
    }
    double time  = timer.get();
    boolean timerReset = RobotState.getInstance().lightsReset;
    if(time < 2){
      if(time%0.2<0.1) {
        lights.setSides(255, 255, 255);
      } else {
        lights.setSides(0, 0, 0);
      }
    }
    else if(!noteInRobot) {
      noteWasIn = false;
      if(noteSeenByLimelight) {
        lights.setSides(70, 35, 0);
      } else {  
        lights.setSides(0, 0, 0);
      }
    }
    else{
      if(!noteWasIn){
        noteWasIn = true;
        timer.reset();
      }
      lights.setProgress((RobotState.getInstance().ShooterError / 50)-0.2, 50, 0, 0, 0, 50, 0);
    }
    if(noteInRobot&&timerReset) {
      if (time > 2){
        timer.reset();
      } 
      RobotState.getInstance().lightsReset = false;
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
