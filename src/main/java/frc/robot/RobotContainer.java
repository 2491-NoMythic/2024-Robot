// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.settings.Constants.XboxDriver.*;
import static frc.robot.settings.Constants.DriveConstants.*;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import frc.robot.commands.Drive;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

// preferences are information saved on the Rio. They are initialized once, then gotten every time we run the code.
  private DrivetrainSubsystem driveTrain;
  Lights lights;
  private Drive defaultDriveCommand;
  private XboxController driverController;
  private XboxController operatorController;
  private Limelight limelight;
  private SendableChooser<Command> autoChooser;
  private PowerDistribution PDP;

  Alliance currentAlliance;
  BooleanSupplier ZeroGyroSup;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //preferences are initialized IF they don't already exist on the Rio
    Preferences.initBoolean("CompBot", true);
    Preferences.initBoolean("Lights", true);
    Preferences.initBoolean("Detector Limelight", false);
    Preferences.initBoolean("Use Limelight", true);
    Preferences.initBoolean("Use 2 Limelights", true);

    DataLogManager.start(); //Start logging
    DriverStation.startDataLog(DataLogManager.getLog()); //Joystick Data logging
    //instantiate both controllers and the PDP
    driverController = new XboxController(DRIVE_CONTROLLER_ID);
    operatorController = new XboxController(OPERATOR_CONTROLLER_ID);
    PDP = new PowerDistribution(1, ModuleType.kRev);
    //this is the supplier for zero-ing the drivetrain's pigeon (gyroscope)
    ZeroGyroSup = ()->driverController.getStartButton();
    
    limelightInit();
    driveTrainInst();
    
    configureDriveTrain();
    configureBindings();
    autoInit();
  }
 
  private void driveTrainInst() {
    driveTrain = new DrivetrainSubsystem();
    defaultDriveCommand = new Drive(
      driveTrain, 
      () -> false,
      () -> modifyAxis(-driverController.getRawAxis(Y_AXIS), DEADBAND_NORMAL),
      () -> modifyAxis(-driverController.getRawAxis(X_AXIS), DEADBAND_NORMAL),
      () -> modifyAxis(-driverController.getRawAxis(Z_AXIS), DEADBAND_NORMAL));
      driveTrain.setDefaultCommand(defaultDriveCommand);
  }

  private void autoInit() {
    registerNamedCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }
  private void limelightInit() {
    limelight = Limelight.getInstance();
  }
  private void lightsInst() {
    lights = new Lights();
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
    SmartDashboard.putData("drivetrain", driveTrain);
    new Trigger(ZeroGyroSup).onTrue(new InstantCommand(driveTrain::zeroGyroscope));
    //this is the command that sets the offsets for each swerve module, so that they can all drive in the same direction line
    InstantCommand setOffsets = new InstantCommand(driveTrain::setEncoderOffsets) {
      public boolean runsWhenDisabled() {
        return true;
      };
    };
    SmartDashboard.putData("set offsets", setOffsets);
    SmartDashboard.putData(new InstantCommand(driveTrain::forceUpdateOdometryWithVision));
  }

  private double modifyAxis(double value, double deadband) {
    // Deadband
    value = MathUtil.applyDeadband(value, deadband);
    // Square the axis
    value = Math.copySign(value * value, value);
    return value;
  }
/**
 * configures the drivetrain for the AutoBuilder. aka gives PathPlanner all the data it needs to drive our robot accurately
 */
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
                ()->DriverStation.getAlliance().get().equals(Alliance.Red),
                driveTrain
    );
  }

  
  private void registerNamedCommands() {}
    
  
  public void logPower(){
    for(int i = 0; i < 16; i++) { 
      SmartDashboard.putNumber("PDP Current " + i, PDP.getCurrent(i));
    }
  }

    /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public void autonomousInit() {
    SmartDashboard.putNumber("autos ran", SmartDashboard.getNumber("autos ran", 0)+1);
  }
  
  public void teleopInit() {}

  public void teleopPeriodic() {
    SmartDashboard.putData(driveTrain.getCurrentCommand());
  }

  public void robotPeriodic() {
      currentAlliance = DriverStation.getAlliance().get();
      SmartDashboard.putString("AlliancePeriodic", currentAlliance == null? "null" : currentAlliance == Alliance.Red? "Red": "Blue" );

      if(Preferences.getBoolean("Use Limelight", false)) {
        limelight.updateLoggingWithPoses();
      }
  }
  public void disabledPeriodic() {}

  public void disabledInit() {
  }
}
