// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.goToPose;

import static frc.robot.settings.Constants.DriveConstants.DEFAUL_PATH_CONSTRAINTS;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.PathConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class GoToNearestClimbSpot extends Command {
  Command actualCommand;
  DrivetrainSubsystem drivetrain;
  SendableChooser<String> climbSpotChooser;
  /** Creates a new GoToAmp. */
  public GoToNearestClimbSpot(DrivetrainSubsystem drivetrain, SendableChooser<String> climbSpotChooser) {
    this.climbSpotChooser = climbSpotChooser;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (climbSpotChooser.getSelected()) {
      case "L-Chain Left":
        System.out.println("L-Chain Left!");
        actualCommand = AutoBuilder.pathfindToPose(PathConstants.CLIMB_LL_POSE, DEFAUL_PATH_CONSTRAINTS);        
        break;    
       case "L-Chain Middle":
        System.out.println("L-Chain Middle!");
        actualCommand = AutoBuilder.pathfindToPose(PathConstants.CLIMB_LM_POSE, DEFAUL_PATH_CONSTRAINTS);        
        break;
       case "L-Chain Right":
        System.out.println("L-Chain Right!");
        actualCommand = AutoBuilder.pathfindToPose(PathConstants.CLIMB_LR_POSE, DEFAUL_PATH_CONSTRAINTS);      
      case "Mid-Chain Left":
        System.out.println("Mid-Chain Left!");
        actualCommand = AutoBuilder.pathfindToPose(PathConstants.CLIMB_ML_POSE, DEFAUL_PATH_CONSTRAINTS);        
        break;    
       case "Mid-Chain Middle":
        System.out.println("Mid-Chain Middle!");
        actualCommand = AutoBuilder.pathfindToPose(PathConstants.CLIMB_MM_POSE, DEFAUL_PATH_CONSTRAINTS);        
        break;
       case "Mid-Chain Right":
        System.out.println("Mid-Chain Right!");
        actualCommand = AutoBuilder.pathfindToPose(PathConstants.CLIMB_MR_POSE, DEFAUL_PATH_CONSTRAINTS);    
        break;
      case "R-Chain Left":
        System.out.println("R-Chain Left!");
        actualCommand = AutoBuilder.pathfindToPose(PathConstants.CLIMB_RL_POSE, DEFAUL_PATH_CONSTRAINTS);        
        break;    
       case "R-Chain Middle":
        System.out.println("R-Chain Middle!");
        actualCommand = AutoBuilder.pathfindToPose(PathConstants.CLIMB_RM_POSE, DEFAUL_PATH_CONSTRAINTS);        
        break;
       case "R-Chain Right":
        System.out.println("R-Chain Right!");
        actualCommand = AutoBuilder.pathfindToPose(PathConstants.CLIMB_RR_POSE, DEFAUL_PATH_CONSTRAINTS);  
      default:
      System.out.println("We broke!(Or went to the default command)");
        break;
    }

  actualCommand.initialize();
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    actualCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return actualCommand.isFinished();
  }
}
