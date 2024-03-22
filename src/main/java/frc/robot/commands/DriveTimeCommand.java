// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveTimeCommand extends Command {
  double forwardSpeed;
  double sidewaysSpeed;
  double radiansPerSecond;
  double time;
  Timer timer;
  DrivetrainSubsystem drivetrain;
  /** Creates a new DriveTimeCommand. */
  public DriveTimeCommand(double forwardSpeed, double sidewaysSpeed, double radiansPerSecond, double time, DrivetrainSubsystem drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.forwardSpeed = forwardSpeed;
    this.sidewaysSpeed = sidewaysSpeed;
    this.radiansPerSecond = radiansPerSecond;
    this.time = time;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    timer.reset();
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  drivetrain.drive(new ChassisSpeeds(
    forwardSpeed, 
    sidewaysSpeed,
    radiansPerSecond
  ));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.reset();
    timer.stop();
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get()>=time);
  }
}
