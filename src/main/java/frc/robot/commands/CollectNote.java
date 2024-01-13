// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.settings.LimelightDetectorData;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.settings.Constants.DriveConstants.*;

public class CollectNote extends Command {
  DrivetrainSubsystem drivetrain;
  Intake intake;
  Limelight limelightClass;
  PIDController txController;
  PIDController taController;
  LimelightDetectorData detectorData;
  double tx;
  double ta;
  /** Creates a new CollectNote. */
  public CollectNote(DrivetrainSubsystem drivetrain, Intake intake, Limelight ll) {
    addRequirements(drivetrain, intake);
    this.drivetrain = drivetrain;
    this.intake = intake;
    this.limelightClass = ll;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    txController = new PIDController(
        K_DETECTOR_TX_P,
        K_DETECTOR_TX_I,
        K_DETECTOR_TX_D);
    taController = new PIDController(
        K_DETECTOR_TA_P,
        K_DETECTOR_TA_I,
        K_DETECTOR_TA_D);
    txController.setSetpoint(0);
    taController.setSetpoint(0.5);
    txController.setTolerance(1, 0.25);
    taController.setTolerance(1, 0.25);

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
    ta = detectorData.ta;

    SmartDashboard.putNumber("Ttx", txController.calculate(tx));
    SmartDashboard.putNumber("Tty", taController.calculate(ta));

    if (taController.atSetpoint() && txController.atSetpoint()) {
      drivetrain.stop();
    } else {
      drivetrain.drive(new ChassisSpeeds(taController.calculate(1/ta), 0, txController.calculate(tx)));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (taController.atSetpoint() && txController.atSetpoint());  }
}
