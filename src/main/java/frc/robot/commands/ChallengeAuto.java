package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ChallengeAuto extends SequentialCommandGroup{
    DrivetrainSubsystem m_drivetrain;
    public ChallengeAuto(DrivetrainSubsystem drivetrain){
        m_drivetrain = drivetrain;
        Double spacer = 1.0;
        addCommands(
        new MoveMeters(drivetrain, 1.5, 0.5, 0, 0),
        new RotateRobot(drivetrain, ()->45), 
        new WaitCommand(()->spacer),
        new RotateRobot(drivetrain, ()->45), 
        new WaitCommand(()->spacer),
        new RotateRobot(drivetrain, ()->45),
        new WaitCommand(()->spacer), 
        new RotateRobot(drivetrain, ()->45), 
        new WaitCommand(()->spacer),    
        new MoveMeters(drivetrain, 1.5, 0.5, 0, 0)
        );
    }
}
