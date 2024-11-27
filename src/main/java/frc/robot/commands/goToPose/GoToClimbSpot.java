// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.goToPose;

import static frc.robot.settings.Constants.DriveConstants.DEFAULT_PATH_CONSTRAINTS;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class GoToClimbSpot extends Command {
  Command actualCommand;
  DrivetrainSubsystem drivetrain;
  SendableChooser<String> climbSpotChooser;
  /** Creates a new GoToAmp. */
  public GoToClimbSpot(DrivetrainSubsystem drivetrain, SendableChooser<String> climbSpotChooser) {
    this.climbSpotChooser = climbSpotChooser;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PathPlannerPath climbMidRPath = PathPlannerPath.fromPathFile("climbMidR");
    PathPlannerPath climbMidMPath = PathPlannerPath.fromPathFile("climbMidM");
    PathPlannerPath climbMidLPath = PathPlannerPath.fromPathFile("climbMidL");
    PathPlannerPath climbSourceSideRPath = PathPlannerPath.fromPathFile("climbSourceSideR");
    PathPlannerPath climbSourceSideMPath = PathPlannerPath.fromPathFile("climbSourceSideM");
    PathPlannerPath climbSourceSideLPath = PathPlannerPath.fromPathFile("climbSourceSideL");
    PathPlannerPath climbAmpSideRPath = PathPlannerPath.fromPathFile("climbAmpSideR");
    PathPlannerPath climbAmpSideMPath = PathPlannerPath.fromPathFile("climbAmpSideM");
    PathPlannerPath climbAmpSideLPath = PathPlannerPath.fromPathFile("climbAmpSideL");
    switch (climbSpotChooser.getSelected()) {
      case "L-Chain Left":
        System.out.println("L-Chain Source!");
        actualCommand = AutoBuilder.pathfindThenFollowPath(climbSourceSideLPath, DEFAULT_PATH_CONSTRAINTS);        
        break;    
       case "L-Chain Middle":
        System.out.println("L-Chain Middle!");
        actualCommand = AutoBuilder.pathfindThenFollowPath(climbMidLPath, DEFAULT_PATH_CONSTRAINTS);        
        break;
       case "L-Chain Right":
        System.out.println("L-Chain Amp!");
        actualCommand = AutoBuilder.pathfindThenFollowPath(climbAmpSideLPath, DEFAULT_PATH_CONSTRAINTS);      
      case "Mid-Chain Left":
        System.out.println("Mid-Chain Source!");
        actualCommand = AutoBuilder.pathfindThenFollowPath(climbSourceSideMPath, DEFAULT_PATH_CONSTRAINTS);        
        break;    
       case "Mid-Chain Middle":
        System.out.println("Mid-Chain Middle!");
        actualCommand = AutoBuilder.pathfindThenFollowPath(climbMidMPath, DEFAULT_PATH_CONSTRAINTS);        
        break;
       case "Mid-Chain Right":
        System.out.println("Mid-Chain Amp!");
        actualCommand = AutoBuilder.pathfindThenFollowPath(climbAmpSideMPath, DEFAULT_PATH_CONSTRAINTS);    
        break;
      case "R-Chain Left":
        System.out.println("R-Chain Source!");
        actualCommand = AutoBuilder.pathfindThenFollowPath(climbSourceSideRPath, DEFAULT_PATH_CONSTRAINTS);        
        break;    
       case "R-Chain Middle":
        System.out.println("R-Chain Middle!");
        actualCommand = AutoBuilder.pathfindThenFollowPath(climbMidRPath, DEFAULT_PATH_CONSTRAINTS);        
        break;
       case "R-Chain Right":
        System.out.println("R-Chain Amp!");
        actualCommand = AutoBuilder.pathfindThenFollowPath(climbAmpSideRPath, DEFAULT_PATH_CONSTRAINTS);
        break; 
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
