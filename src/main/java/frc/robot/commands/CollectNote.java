// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.Vision;
import frc.robot.settings.LimelightDetectorData;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;

public class CollectNote extends Command {

  DrivetrainSubsystem drivetrain;
  LimelightDetectorData detectorData;
  Limelight limelight;
  double runsInvalid;

  PIDController txController;
  PIDController tyController;
  SlewRateLimiter tyLimiter;
  Boolean closeNote;
  double tx;
  double ty;
  /** Creates a new CollectNote. */
  public CollectNote(DrivetrainSubsystem drivetrain, Limelight limelight) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    runsInvalid = 0;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    runsInvalid = 0;
    closeNote = false;
    txController = new PIDController(
        // Vision.K_DETECTOR_TX_P,
        0.035,//0.03,
        Vision.K_DETECTOR_TX_I,
        Vision.K_DETECTOR_TX_D);
    tyController = new PIDController(
        0.6,//Vision.K_DETECTOR_TY_P,
        Vision.K_DETECTOR_TY_I,
        Vision.K_DETECTOR_TY_D);
    tyLimiter = new SlewRateLimiter(20, -20, 0);
    txController.setSetpoint(0);
    tyController.setSetpoint(0);
    txController.setTolerance(3.5, 0.25);
    tyController.setTolerance(1, 0.25);
    detectorData = limelight.getNeuralDetectorValues();
    SmartDashboard.putBoolean("note seen", detectorData.isResultValid);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    detectorData = limelight.getNeuralDetectorValues();

    if (detectorData == null) {
      drivetrain.stop();
      System.err.println("nullDetectorData");
      return;
    }
    if (!detectorData.isResultValid) {
      if (runsInvalid <= 10) { // don't stop imediately, in case only a couple frames were missed
        drivetrain.drive(new ChassisSpeeds(tyLimiter.calculate(0), 0, 0));
      } else {
        drivetrain.stop();
      }
      System.err.println("invalidDetectorData");
      runsInvalid++;
      return;
    } else {
      runsInvalid = 0;
    }
    
    ty = detectorData.ty;
    if(!closeNote) {
      tx = detectorData.tx;
      double forwardSpeed = tyLimiter.calculate(-20/Math.abs(tx));
      SmartDashboard.putNumber("CollectNote/forward speed limited", forwardSpeed);
      if(Math.abs(forwardSpeed)>1) {forwardSpeed = -1;}
      drivetrain.drive(new ChassisSpeeds(
        forwardSpeed,
        txController.calculate(-tx),
        0));
      } else if(detectorData.ty<=5.5){
        tx = detectorData.tx;
        double forwardSpeed = tyLimiter.calculate(-20/Math.abs(tx));
        if(Math.abs(forwardSpeed)>1) {forwardSpeed = -1;}
        drivetrain.drive(new ChassisSpeeds(
          forwardSpeed,
          txController.calculate(-tx),
          0));
        SmartDashboard.putNumber("CollectNote/forward speed limited", forwardSpeed);
    } else {
      drivetrain.drive(new ChassisSpeeds(
        -1, 0, 0));
        runsInvalid++;
    }
    // else {
    //   runsInvalid++;
    // }
    if (ty<5.5){
      closeNote = true;
    } 
    
    SmartDashboard.putNumber("CollectNote/calculated radians per second", txController.calculate(tx));
    SmartDashboard.putNumber("CollectNote/calculated forward meters per second", tyLimiter.calculate(-20/Math.abs(tx)));
    SmartDashboard.putBoolean("CollectNote/closeNote", closeNote);
    SmartDashboard.putNumber("CollectNote/runsInvalid", runsInvalid);
    // drives the robot forward faster if the object is higher up on the screen, and turns it more based on how far away the object is from x=0
  }
  


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // drivetrain.pointWheelsInward();
    runsInvalid = 0;
    closeNote = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((tyController.atSetpoint() && txController.atSetpoint()) || detectorData == null || runsInvalid>5); 
  }
}
