// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.settings.LimelightDetectorData;
import frc.robot.subsystems.DrivetrainSubsystem;
import static frc.robot.settings.Constants.DriveConstants.*;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;

public class CollectNote extends Command {

  DrivetrainSubsystem drivetrain;
  IntakeSubsystem intake;
  LimelightDetectorData detectorData;
  Limelight limelight;

  PIDController txController;
  PIDController tyController;

  double tx;
  double ty;
  /** Creates a new CollectNote. */
  public CollectNote(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, Limelight limelight) {
    addRequirements(drivetrain, intake);
    this.drivetrain = drivetrain;
    this.intake = intake;
    this.limelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    txController = new PIDController(
        K_DETECTOR_TX_P,
        K_DETECTOR_TX_I,
        K_DETECTOR_TX_D);
    tyController = new PIDController(
        K_DETECTOR_TA_P,
        K_DETECTOR_TA_I,
        K_DETECTOR_TA_D);
    txController.setSetpoint(0);
    tyController.setSetpoint(0);
    txController.setTolerance(1, 0.25);
    tyController.setTolerance(1, 0.25);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     detectorData = Limelight.latestDetectorValues;

    if (detectorData == null) {
      drivetrain.stop();
      System.err.println("nullDetectorData");
      return;
    }
    if (!detectorData.isResultValid) {

      drivetrain.stop();
      System.err.println("invalidDetectorData");
      return;
    }
    
    tx = detectorData.tx;
    ty = detectorData.ty;
    
    //drives the robot forward faster if the object is higher up on the screen, and turns it more based on how far away the object is from x=0
    drivetrain.drive(new ChassisSpeeds(
      0,
      -tyController.calculate(ty),
      txController.calculate(tx)));
  }
  


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((tyController.atSetpoint() && txController.atSetpoint()) || detectorData == null || !detectorData.isResultValid); 
  }
}
