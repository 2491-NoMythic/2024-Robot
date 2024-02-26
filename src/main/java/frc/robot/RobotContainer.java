// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.settings.Constants.PS4Driver.*;
import static frc.robot.settings.Constants.ShooterConstants.LONG_SHOOTING_RPS;
import static frc.robot.settings.Constants.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import frc.robot.commands.AimShooter;
import frc.robot.commands.AimRobotMoving;
import frc.robot.commands.CollectNote;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveTimeCommand;
import frc.robot.commands.ConditionalIndexer;
import frc.robot.settings.Constants.IndexerConstants;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.IndicatorLights;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.ClimberConstants;
import frc.robot.settings.Constants.IntakeConstants;
import frc.robot.settings.Constants.ShooterConstants;
import frc.robot.subsystems.AngleShooterSubsystem;
import frc.robot.subsystems.Climber;
import frc.robot.commands.ManualShoot;
import frc.robot.commands.NamedCommands.initialShot;
import frc.robot.commands.NamedCommands.shootNote;
import frc.robot.commands.goToPose.GoToAmp;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RobotState;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.commands.AngleShooter;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

// preferences are information saved on the Rio. They are initialized once, then gotten every time we run the code.
  private final boolean intakeExists = Preferences.getBoolean("Intake", true);
  private final boolean shooterExists = Preferences.getBoolean("Shooter", true);
  private final boolean angleShooterExists = Preferences.getBoolean("AngleShooter", true);
  private final boolean climberExists = Preferences.getBoolean("Climber", true);
  private final boolean lightsExist = Preferences.getBoolean("Lights", true);
  private final boolean indexerExists = Preferences.getBoolean("Indexer", true);
  //private final boolean autosExist = Preferences.getBoolean("Autos", true);
  private final boolean useDetectorLimelight = Preferences.getBoolean("Detector Limelight", true);

  private DrivetrainSubsystem driveTrain;
  private IntakeSubsystem intake;
  private ShooterSubsystem shooter;
  private AngleShooterSubsystem angleShooterSubsystem;
  private Drive defaultDriveCommand;
  private Climber climber;
  private Lights lights;
  private PS4Controller driverController;
  private PS4Controller operatorController;
  //private PS4Controller operatorController;
  private Limelight limelight;
  private IndexCommand defaulNoteHandlingCommand;
  private IndexerSubsystem indexer;
  private AimShooter defaultShooterAngleCommand;
  private SendableChooser<String> climbSpotChooser;
  private SendableChooser<Command> autoChooser;
  private PowerDistribution PDP;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //preferences are initialized IF they don't already exist on the Rio
    Preferences.initBoolean("Brushes", false);
    Preferences.initBoolean("Intake", false);
    Preferences.initBoolean("Climber", false);
    Preferences.initBoolean("Shooter", false);
    Preferences.initBoolean("AngleShooter", false);
    Preferences.initBoolean("Lights", false);
    Preferences.initBoolean("Indexer", false);
    Preferences.initBoolean("Detector Limelight", false);
    Preferences.initBoolean("Use Limelight", false);
    Preferences.initBoolean("Use 2 Limelights", false);
    Preferences.initDouble("wait # of seconds", 0);

    // DataLogManager.start();
    // URCL.start();
    // SignalLogger.setPath("/media/sda1/ctre-logs/");
    // SignalLogger.start();
    driverController = new PS4Controller(DRIVE_CONTROLLER_ID);
    operatorController = new PS4Controller(OPERATOR_CONTROLLER_IDds);
    //operatorController = new PS4Controller(OPERATOs_CONTROLLER_ID);
    PDP = new PowerDistribution(1, ModuleType.kRev);
    
    // = new PathPlannerPath(null, DEFAUL_PATH_CONSTRAINTS, null, climberExists);
    limelightInit();
    driveTrainInst();
    

    if(intakeExists) {intakeInst();}
    if(shooterExists) {shooterInst();}
    if(angleShooterExists) {angleShooterInst();}
    if(climberExists) {climberInst();}
    climbSpotChooserInit();
    if(lightsExist) {lightsInst();}
    if(indexerExists) {indexInit();}
    if(intakeExists && shooterExists && indexerExists && angleShooterExists) {indexCommandInst();}
    Limelight.useDetectorLimelight(useDetectorLimelight);
    autoInit();
    // Configure the trigger bindings
    configureBindings();
  }
  private void climbSpotChooserInit() {
    climbSpotChooser = new SendableChooser<String>();
    climbSpotChooser.addOption("Climb L-Chain Amp", "L-Chain Amp");
    climbSpotChooser.addOption("Climb L-Chain Middle", "L-Chain Middle");
    climbSpotChooser.addOption("Climb L-Chain Source", "L-Chain Source");

    climbSpotChooser.addOption("Climb Mid-Chain Source", "Mid-Chain Source");
    climbSpotChooser.addOption("Climb Mid-Chain Amp", "Mid-Chain Amp");
    climbSpotChooser.addOption("Climb Mid-Chain Middle", "Mid-Chain Middle");

    climbSpotChooser.addOption("Climb R-Chain Source", "R-Chain Source");
    climbSpotChooser.addOption("Climb R-Chain Right", "R-Chain Right");
    climbSpotChooser.addOption("Climb R-Chain Amp", "R-Chain Amp");
    SmartDashboard.putData(climbSpotChooser);
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
    shooter = new ShooterSubsystem(ShooterConstants.SHOOTER_MOTOR_POWER);
  }
  private void angleShooterInst(){
    angleShooterSubsystem = new AngleShooterSubsystem();
    defaultShooterAngleCommand = new AimShooter(angleShooterSubsystem, driverController::getPOV, driverController::getR1Button, driverController::getCrossButton);
    angleShooterSubsystem.setDefaultCommand(defaultShooterAngleCommand);
  }
  private void intakeInst() {
    intake = new IntakeSubsystem();
  }
  private void climberInst() {
    climber = new Climber();
  }
  private void indexInit() {
    indexer = new IndexerSubsystem();
  }
  private void indexCommandInst() {
    defaulNoteHandlingCommand = new IndexCommand(indexer, driverController::getR2Button, driverController::getL2Button, shooter, intake, driveTrain, angleShooterSubsystem, driverController::getR1Button);
    indexer.setDefaultCommand(defaulNoteHandlingCommand);
  }

  private void autoInit() {
    configureDriveTrain();
    registerNamedCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }
  private void limelightInit() {
    limelight = Limelight.getInstance();
  }
  private void lightsInst() {
    lights = new Lights(Constants.LED_COUNT-1);
    lights.setDefaultCommand(new IndicatorLights(lights));
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
    // new Trigger(driverController::getCrossButton).onTrue(new autoAimParallel(driveTrain/*, shooter*/));
    new Trigger(driverController::getPSButton).onTrue(new InstantCommand(driveTrain::zeroGyroscope));
    // new Trigger(driverController::getCircleButton).whileTrue(new GoToAmp(driveTrain)); unused becuase we dont pickup from amp with a path anymore
    new Trigger(driverController::getL2Button).whileTrue(new AimRobotMoving(
      driveTrain,
      () -> modifyAxis(-driverController.getRawAxis(Z_AXIS), DEADBAND_NORMAL),
      () -> modifyAxis(-driverController.getRawAxis(Y_AXIS), DEADBAND_NORMAL),
      () -> modifyAxis(-driverController.getRawAxis(X_AXIS), DEADBAND_NORMAL),
      driverController::getL2Button,
      driverController::getCrossButton
      driverController::getTriangleButton));

    if(Preferences.getBoolean("Detector Limelight", false)) {
      new Trigger(operatorController::getR1Button).onTrue(new SequentialCommandGroup(
        new CollectNote(driveTrain, limelight),
        new DriveTimeCommand(-2, 0, 0, 0.5, driveTrain)
      ));
    }
    new Trigger(driverController::getOptionsButton).onTrue(new InstantCommand(()->SmartDashboard.putBoolean("force use limelight", true))).onFalse(new InstantCommand(()->SmartDashboard.putBoolean("force use limelight", false)));
    new Trigger(driverController::getTouchpadPressed).onTrue(new InstantCommand(driveTrain::stop, driveTrain));
    SmartDashboard.putData("force update limelight position", new InstantCommand(()->driveTrain.forceUpdateOdometryWithVision(), driveTrain));
    if(angleShooterExists) {
      new Trigger(()->driverController.getPOV() == 180).whileTrue(new AngleShooter(angleShooterSubsystem, ()->ShooterConstants.MAXIMUM_SHOOTER_ANGLE));
      SmartDashboard.putData("Manual Angle Shooter Up", new AngleShooter(angleShooterSubsystem, ()->ShooterConstants.MAXIMUM_SHOOTER_ANGLE));
    }
    if(indexerExists) {
      new Trigger(driverController::getL1Button).whileTrue(new ManualShoot(indexer, driverController::getPOV));
    }
    if(climberExists) {
      // new Trigger(driverController::getCrossButton).whileTrue(new AutoClimb(climber)).onFalse(new InstantCommand(()-> climber.climberStop()));
      new Trigger(operatorController::getCrossButton).onTrue(new InstantCommand(()-> climber.climberGo(ClimberConstants.CLIMBER_SPEED_DOWN))).onFalse(new InstantCommand(()-> climber.climberGo(0)));
      new Trigger(operatorController::getTriangleButton).onTrue(new InstantCommand(()-> climber.climberGo(ClimberConstants.CLIMBER_SPEED_UP))).onFalse(new InstantCommand(()-> climber.climberGo(0)));
      // new Trigger(driverController::getSquareButton).whileTrue(new ClimberPullDown(climber));
    }
    if(shooterExists) {
      new Trigger(()->driverController.getPOV() == 90).whileTrue(new InstantCommand(()->shooter.shootRPS(ShooterConstants.AMP_RPS), shooter));
    }
    if(intakeExists) {
      new Trigger(driverController::getTouchpad).onTrue(new InstantCommand(()->intake.intakeYes(IntakeConstants.INTAKE_SPEED))).onFalse(new InstantCommand(intake::intakeOff));
    }
    if(indexerExists&&shooterExists&&angleShooterExists) {}
    InstantCommand setOffsets = new InstantCommand(driveTrain::setEncoderOffsets) {
      public boolean runsWhenDisabled() {
        return true;
      };
    };
    SmartDashboard.putData("set offsets", setOffsets);
    SmartDashboard.putData(new InstantCommand(driveTrain::forceUpdateOdometryWithVision));
/* bindings:
 *    L2: aim at speaker and rev up shooter to max (hold)
 *    L1: manually feed shooter (hold)
 *    R2: shoot if everything is lined up (hold)
 *    Circle: lineup with the amp +shoot at amp speed (hold)
 *    D-Pad down: move shooter up manually (hold)
 *    R1: aim shooter at amp (hold)
 *    Options button: collect note from human player
 *    Square: auto-pick up note
 *    Touchpad: manually turn on Intake (hold) [only works if intake code doesn't exist in IndexCommand]
 *    L1,L2,R1,R2 held: aim shooter at speaker and set shooter to shooter speed
 * 
 *  operator:
 *    Triangle: climber up (hold)
 *    Cross: climber down (hold)
 *    R1: auto pickup note from ground (hold)
 *    
 */
//FOR TESTING PURPOSES:
    if(intakeExists) {
      SmartDashboard.putData("intake on", new InstantCommand(()->intake.intakeYes(IntakeConstants.INTAKE_SPEED), intake));
      SmartDashboard.putData("intake off", new InstantCommand(intake::intakeOff, intake));
    }
    if(shooterExists) {
      SmartDashboard.putData("shooter on speaker", new InstantCommand(()->shooter.shootRPS(ShooterConstants.LONG_SHOOTING_RPS), shooter));
      SmartDashboard.putData("shooter on amp", new InstantCommand(()->shooter.shootRPS(ShooterConstants.AMP_RPS), shooter));
      SmartDashboard.putData("shooter off", new InstantCommand(shooter::turnOff, shooter));
    }
    if(angleShooterExists) {
      double testAngle = 45;
      SmartDashboard.putData("go to angle", new AngleShooter(angleShooterSubsystem, ()->testAngle));
      SmartDashboard.putData("run indexer down slow", new InstantCommand(()->angleShooterSubsystem.pitchShooter(0.02), angleShooterSubsystem));
      SmartDashboard.putData("run indexer up slow", new InstantCommand(()->angleShooterSubsystem.pitchShooter(-0.02), angleShooterSubsystem));
      SmartDashboard.putData("stop pitch", new InstantCommand(()->angleShooterSubsystem.pitchShooter(0), angleShooterSubsystem));
      SmartDashboard.putData("set reference to 60", new InstantCommand(()->angleShooterSubsystem.setDesiredShooterAngle(60)));
      SmartDashboard.putData("set reference to 30", new InstantCommand(()->angleShooterSubsystem.setDesiredShooterAngle(45)));
      SmartDashboard.putData("set reference to 15", new InstantCommand(()->angleShooterSubsystem.setDesiredShooterAngle(17)));
    }
    if(indexerExists) {
      SmartDashboard.putData("indexer intake speed", new InstantCommand(()->indexer.set(IndexerConstants.INDEXER_INTAKE_SPEED)));
      SmartDashboard.putData("indexer shooting speed", new InstantCommand(()->indexer.set(IndexerConstants.INDEXER_SHOOTING_SPEED)));
      SmartDashboard.putData("indexer off", new InstantCommand(()->indexer.off()));
    }
    if(climberExists) {
      SmartDashboard.putData("climber up", new InstantCommand(()-> climber.climberGo(ClimberConstants.CLIMBER_SPEED_UP)));
      SmartDashboard.putData("climber down", new InstantCommand(()-> climber.climberGo(-ClimberConstants.CLIMBER_SPEED_UP)));
      SmartDashboard.putData("climber stop", new InstantCommand(()-> climber.climberStop()));
    }
  };


    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
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
                ()->DriverStation.getAlliance().get().equals(Alliance.Red),
                driveTrain
    );
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("stopDrivetrain", new InstantCommand(driveTrain::stop, driveTrain));
    NamedCommands.registerCommand("autoPickup", new CollectNote(driveTrain, limelight));

    if(shooterExists) {NamedCommands.registerCommand("shooterOn", new InstantCommand(()->shooter.shootRPS(LONG_SHOOTING_RPS), shooter));}
    if(indexerExists) {NamedCommands.registerCommand("feedShooter", new InstantCommand(()->indexer.set(IndexerConstants.INDEXER_SHOOTING_SPEED), indexer));
    NamedCommands.registerCommand("stopFeedingShooter", new InstantCommand(indexer::off, indexer));}
    if(intakeExists) {
      NamedCommands.registerCommand("intakeOn", new InstantCommand(()-> intake.intakeYes(1)));
    }
    if(indexerExists&&shooterExists) {
      NamedCommands.registerCommand("initialShot", new initialShot(shooter, indexer, 0.75, 0.5));
      NamedCommands.registerCommand("shootNote", new shootNote(indexer, 1));
      NamedCommands.registerCommand("shootNote", new shootNote(indexer, 1));
      NamedCommands.registerCommand("setFeedTrue", new InstantCommand(()->SmartDashboard.putBoolean("feedMotor", true)));
      NamedCommands.registerCommand("setFeedFalse", new InstantCommand(()->SmartDashboard.putBoolean("feedMotor", false)));
    }
    if (indexerExists&&intakeExists) {
      NamedCommands.registerCommand("conditionalindexer", new ConditionalIndexer(indexer,intake));
    }
    NamedCommands.registerCommand("wait x seconds", new WaitCommand(Preferences.getDouble("wait # of seconds", 0)));
  }
  public void teleopInit() {
    driveTrain.forceUpdateOdometryWithVision();
    if(climberExists) {
      SequentialCommandGroup resetClimbers = new SequentialCommandGroup(
        new InstantCommand(()->climber.climberGo(ClimberConstants.CLIMBER_SPEED_DOWN), climber),
        new WaitCommand(2),
        new InstantCommand(()->climber.climberStop(), climber),
        new InstantCommand(climber::resetInitial)
        );
      resetClimbers.schedule();
      }
      if(angleShooterExists) {
        angleShooterSubsystem.pitchShooter(0);
      }
  }
  public void teleopPeriodic() {
    SmartDashboard.putData(driveTrain.getCurrentCommand());
    driveTrain.calculateSpeakerAngle();
    if(useDetectorLimelight) {
      SmartDashboard.putNumber("Is Note Seen?", limelight.getNeuralDetectorValues().ta);
      RobotState.getInstance().IsNoteSeen = limelight.getNeuralDetectorValues().isResultValid;
    }
    SmartDashboard.putBoolean("is note seen", RobotState.getInstance().IsNoteSeen);
		SmartDashboard.putBoolean("shooter in range", RobotState.getInstance().ShooterInRange);
		SmartDashboard.putBoolean("shooter ready", RobotState.getInstance().ShooterReady);
  }
 
  public void logPower(){
    for(int i = 0; i < 16; i++) { 
      SmartDashboard.putNumber("PDP Current " + i, PDP.getCurrent(i));
    }
  }
  public void robotPeriodic() {
    logPower();
  }
  public void disabledPeriodic() {
  
  }

  public void disabledInit() {
    if(angleShooterExists) {
      new InstantCommand(angleShooterSubsystem::stop, angleShooterSubsystem);
    }
  }
}
