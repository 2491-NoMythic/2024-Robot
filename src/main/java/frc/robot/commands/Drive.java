package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Drive extends Command {
    private final DrivetrainSubsystem drivetrain;
    private final BooleanSupplier robotCentricMode;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;
    private int invert;
    /**
     * drives the robot at a specific forward velocity, sideways velocity, and rotational velocity.
     * @param drivetrainSubsystem Swerve drive subsytem
     * @param robotCentricMode while this is pressed, the robot will drive in RobotCentric mode. Otherwise, it will default to field centric
     * @param translationXSupplier forward throttle (from -1 to 1). 1 will drive at full speed forward
     * @param translationYSupplier sideways throttle (from -1 to 1). 1 will drive at full speed to the right
     * @param rotationSupplier rotational throttle (from -1 to 1). 1 will drive at full speed clockwise
     */
    public Drive(DrivetrainSubsystem drivetrainSubsystem,
    BooleanSupplier robotCentricMode,
    DoubleSupplier translationXSupplier,
    DoubleSupplier translationYSupplier,
    DoubleSupplier rotationSupplier) {
        this.drivetrain = drivetrainSubsystem;
        this.robotCentricMode = robotCentricMode;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;
        addRequirements(drivetrainSubsystem);
    }
    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        if(DriverStation.getAlliance().get() == Alliance.Red) {
            invert = -1;
        } else {
            invert = 1;
        }
        if (robotCentricMode.getAsBoolean()) {
            drivetrain.drive(new ChassisSpeeds(
                translationXSupplier.getAsDouble() * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND * invert,
                translationYSupplier.getAsDouble() * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND * invert,
                rotationSupplier.getAsDouble() * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
            ));
        } else {
            drivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    translationXSupplier.getAsDouble() * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND * invert,
                    translationYSupplier.getAsDouble() * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND * invert,
                    rotationSupplier.getAsDouble() * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                    drivetrain.getPose().getRotation()
                )
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}