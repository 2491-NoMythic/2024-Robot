// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.settings.Constants.PS4Driver.*;
import static frc.robot.settings.Constants.PS4Operator.*;

import java.util.function.BooleanSupplier;

import static frc.robot.settings.Constants.DriveConstants.*;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import frc.robot.commands.Autos;
import frc.robot.commands.Drive;
import frc.robot.commands.ExampleCommand;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.subsystems.Climber;
import frc.robot.commands.ManualShoot;
import frc.robot.commands.autoAimParallel;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.Drive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import frc.robot.commands.ManualShoot;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

// preferences are information saved on the Rio. They are initialized once, then gotten every time we run the code.
  private final boolean intakeExists = Preferences.getBoolean("Intake", true);
  private final boolean shooterExists = Preferences.getBoolean("Shooter", true);
  private final boolean climberExists = Preferences.getBoolean("Climber", true);

  private DrivetrainSubsystem driveTrain;
  private Intake intake;
  private ShooterSubsystem shooter;
  private Drive defaultDriveCommand;
  private Climber climber;
  private PS4Controller driverController;
  private PS4Controller operatorController;
  private Limelight limelight;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
//preferences are initialized IF they don't already exist on the Rio
    Preferences.initBoolean("Intake", false);
    Preferences.initBoolean("Climber", false);
    Preferences.initBoolean("Shooter", false);

    driverController = new PS4Controller(DRIVE_CONTROLLER_ID);
    operatorController = new PS4Controller(OPERATOR_CONTROLLER_ID);

    driveTrainInst();
    autoInit();
    limelightInit();
    if(intakeExists) {intakeInst();}
    if(shooterExists) {shooterInst();}
    if(climberExists) {climberInst();}
    // Configure the trigger bindings
    configureBindings();
  }

  private void driveTrainInst() {
    driveTrain = new DrivetrainSubsystem();
    defaultDriveCommand = new Drive(
      driveTrain, 
      () -> driverController.getL1Button(),
      () -> modifyAxis(-driverController.getRawAxis(Y_AXIS), DEADBAND_NORMAL),
      () -> modifyAxis(-driverController.getRawAxis(X_AXIS), DEADBAND_NORMAL),
      () -> modifyAxis(-driverController.getRawAxis(Z_AXIS), DEADBAND_NORMAL));
    driveTrain.setDefaultCommand(defaultDriveCommand);
  }
  private void shooterInst() {
    shooter = new ShooterSubsystem(1);
  }
  private void intakeInst() {
    intake = new Intake();
  }
  private void climberInst() {
    climber = new Climber();
  }
  private void autoInit() {
    configureDriveTrain();
    SmartDashboard.putData("Auto Chooser", AutoBuilder.buildAutoChooser());
    registerNamedCommands();
  }
  private void limelightInit() {
    limelight = Limelight.getInstance();
  }
  

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    new Trigger(driverController::getCrossButton).onTrue(new autoAimParallel(driveTrain, shooter));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  // private Command ManualShoot(ShooterSubsystem shooter) {
  //   // TODO Auto-generated method stub
  // }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

  private double modifyAxis(double value, double deadband) {
    // Deadband
    value = MathUtil.applyDeadband(value, deadband);
    // Square the axis
    value = Math.copySign(value * value, value);
    return value;
  }

  private void configureDriveTrain() {
    AutoBuilder.configureHolonomic(
                driveTrain::getPose, // Pose2d supplier
                driveTrain::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
                driveTrain::getChassisSpeeds,
                driveTrain::drive,
                new HolonomicPathFollowerConfig(
                    new PIDConstants(
                        k_XY_P,
                        k_XY_I,
                        k_XY_D), // PID constants to correct for translation error (used to create the X
                                 // and Y PID controllers)
                    new PIDConstants(
                        k_THETA_P,
                        k_THETA_I,
                        k_THETA_D), // PID constants to correct for rotation error (used to create the
                                    // rotation controller)
                    4, //max module speed //TODO find actual values
                    new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0).getNorm(), //drive base radius
                    new ReplanningConfig()
                ),
                ()->DriverStation.getAlliance().equals(Alliance.Red),
                driveTrain
    );
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("stopDrivetrain", new InstantCommand(driveTrain::stop, driveTrain));
  }
}
