// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.settings.Constants.PS4Driver.*;
import static frc.robot.settings.Constants.ShooterConstants.PRAC_AMP_RPS;
import static frc.robot.settings.Constants.ShooterConstants.LONG_SHOOTING_RPS;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.management.InstanceNotFoundException;

import static frc.robot.settings.Constants.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import frc.robot.commands.AimShooter;
import frc.robot.commands.AimRobotMoving;
import frc.robot.commands.CollectNote;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveTimeCommand;
import frc.robot.commands.GroundIntake;
import frc.robot.commands.NamedCommands.AutoGroundIntake;
import frc.robot.settings.Constants.IndexerConstants;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.IndexerNoteAlign;
import frc.robot.commands.IndicatorLights;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.ClimberConstants;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.settings.Constants.Field;
import frc.robot.settings.Constants.IntakeConstants;
import frc.robot.settings.Constants.ShooterConstants;
import frc.robot.subsystems.AngleShooterSubsystem;
import frc.robot.subsystems.Climber;
import frc.robot.commands.ManualShoot;
import frc.robot.commands.MoveMeters;
import frc.robot.commands.OverrideCommand;
import frc.robot.commands.PodiumCollectNote;
import frc.robot.commands.WaitUntil;

import frc.robot.commands.NamedCommands.InitialShot;
import frc.robot.commands.NamedCommands.ShootNote;
import frc.robot.commands.NamedCommands.AutoGroundIntake;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.commands.AngleShooter;
import frc.robot.commands.ClimberCommand;


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
  private final boolean sideWheelsExists = Preferences.getBoolean("IntakeSideWheels", true);
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

  Alliance currentAlliance;
  BooleanSupplier ZeroGyroSup;
  BooleanSupplier AimWhileMovingSup;
  BooleanSupplier ShootIfReadySup;
  BooleanSupplier SubwooferAngleSup;
  BooleanSupplier StageAngleSup;
  BooleanSupplier HumanPlaySup;
  BooleanSupplier AmpAngleSup;
  BooleanSupplier SourcePickupSup;
  BooleanSupplier ClimberDownSup;
  BooleanSupplier ShooterUpManualSup;
  BooleanSupplier ManualShootSup;
  BooleanSupplier ForceVisionSup;
  BooleanSupplier GroundIntakeSup;
  BooleanSupplier FarStageAngleSup;
  BooleanSupplier OperatorRevToZero;
  BooleanSupplier OverStagePassSup;
  BooleanSupplier OppositeStageShotSup;
  BooleanSupplier falseSup;
  DoubleSupplier zeroSup;
  BooleanSupplier AutoPickupSup;
  BooleanSupplier CenterAmpPassSup;

  BooleanSupplier intakeReverse;
  Command autoPickup;
  Command podiumAutoPickup;
  // Replace with CommandPS4Controller or CommandJoystick if needed
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //preferences are initialized IF they don't already exist on the Rio
    SmartDashboard.putNumber("amp RPS", ShooterConstants.PRAC_AMP_RPS);

    Preferences.initBoolean("Brushes", false);
    Preferences.initBoolean("CompBot", true);
    Preferences.initBoolean("Climber", true);
    Preferences.initBoolean("Intake", true);
    Preferences.initBoolean("IntakeSideWheels", false);
    Preferences.initBoolean("Shooter", true);
    Preferences.initBoolean("AngleShooter", true);
    Preferences.initBoolean("Lights", true);
    Preferences.initBoolean("Indexer", true);
    Preferences.initBoolean("Detector Limelight", false);
    Preferences.initBoolean("Use Limelight", true);
    Preferences.initBoolean("Use 2 Limelights", true);
    Preferences.initDouble("wait # of seconds", 0);

    DataLogManager.start(); //Start logging
    DriverStation.startDataLog(DataLogManager.getLog()); //Joystick Data logging

    // DataLogManager.start();
    // URCL.start();
    // SignalLogger.setPath("/media/sda1/ctre-logs/");
    // SignalLogger.start();
    driverController = new PS4Controller(DRIVE_CONTROLLER_ID);
    operatorController = new PS4Controller(OPERATOR_CONTROLLER_ID);
    //operatorController = new PS4Controller(OPERATOs_CONTROLLER_ID);
    PDP = new PowerDistribution(1, ModuleType.kRev);

    ZeroGyroSup = driverController::getPSButton;
    ForceVisionSup = driverController::getOptionsButton;

    AimWhileMovingSup = driverController::getL2Button;
    HumanPlaySup = driverController::getR1Button;
    AmpAngleSup = ()->driverController.getPOV() == 90||driverController.getPOV() == 45||driverController.getPOV() == 135;;
    ManualShootSup = driverController::getR2Button;
    ClimberDownSup = operatorController::getPSButton;
    GroundIntakeSup = driverController::getL1Button;
    OperatorRevToZero = ()->operatorController.getPOV() != -1;
    SubwooferAngleSup =()-> driverController.getCrossButton()||operatorController.getCrossButton();
    StageAngleSup = ()->operatorController.getTriangleButton()||driverController.getTriangleButton();;
    FarStageAngleSup = ()->operatorController.getSquareButton()||driverController.getSquareButton();
    OppositeStageShotSup = ()->operatorController.getCircleButton()||driverController.getCircleButton();
    OverStagePassSup = operatorController::getL1Button;
    CenterAmpPassSup = operatorController::getL2Button;
    AutoPickupSup = ()->operatorController.getTouchpad()||driverController.getTouchpad();
    zeroSup = ()->0;
    falseSup = ()->false;
    //discontinued buttons:
    intakeReverse = ()->false;
    ShooterUpManualSup = ()->false;
    ForceVisionSup = ()->false;
    ShootIfReadySup = ()->false;
    
    // = new PathPlannerPath(null, DEFAUL_PATH_CONSTRAINTS, null, climberExists);
    limelightInit();
    driveTrainInst();
    
    if(intakeExists) {intakeInst(); /* Must happen before indexInit */}
    if(shooterExists) {shooterInst();}
    if(angleShooterExists) {angleShooterInst();}
    if(climberExists) {climberInst();}
    climbSpotChooserInit();
    if(lightsExist) {lightsInst();}
    if(indexerExists) {indexInit();}
    if(intakeExists && shooterExists && indexerExists && angleShooterExists) {indexCommandInst();}
    configureDriveTrain();
    configureBindings();
    autoInit();
    ampShotInit();
    // Configure the trigger bindings
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
      () -> false,
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
    defaultShooterAngleCommand = new AimShooter(angleShooterSubsystem, AmpAngleSup, HumanPlaySup, SubwooferAngleSup, StageAngleSup, GroundIntakeSup, FarStageAngleSup, OverStagePassSup, OppositeStageShotSup, CenterAmpPassSup);
    angleShooterSubsystem.setDefaultCommand(defaultShooterAngleCommand);
  }
  private void intakeInst() {
    intake = new IntakeSubsystem();
  }
  private void climberInst() {
    climber = new Climber();
    climber.setDefaultCommand(new ClimberCommand(
      climber,
      ()-> modifyAxis(operatorController.getLeftY(), DEADBAND_NORMAL),
      ()-> modifyAxis(operatorController.getRightY(), DEADBAND_NORMAL),
      ClimberDownSup));
  }
  private void indexInit() {
    indexer = new IndexerSubsystem(intakeExists ? RobotState.getInstance()::isNoteSeen : () -> false);
  }
  private void indexCommandInst() {
    defaulNoteHandlingCommand = new IndexCommand(indexer, ShootIfReadySup, AimWhileMovingSup, shooter, intake, driveTrain, angleShooterSubsystem, HumanPlaySup, StageAngleSup, SubwooferAngleSup, GroundIntakeSup, FarStageAngleSup, OperatorRevToZero, intakeReverse, OverStagePassSup, OppositeStageShotSup);
    indexer.setDefaultCommand(defaulNoteHandlingCommand);
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
    new Trigger(AmpAngleSup).onTrue(new InstantCommand(driveTrain::pointWheelsInward, driveTrain));
    SmartDashboard.putData("drivetrain", driveTrain);
    Command setGyroTo180 = new InstantCommand(()->driveTrain.zeroGyroscope(180)) {
      public boolean runsWhenDisabled() {
              return true;
      };
    };
    SmartDashboard.putData("set gyro 180", setGyroTo180);
    // new Trigger(driverController::getCrossButton).onTrue(new autoAimParallel(driveTrain/*, shooter*/));
    new Trigger(ZeroGyroSup).onTrue(new InstantCommand(driveTrain::zeroGyroscope));
    // new Trigger(driverController::getCircleButton).whileTrue(new GoToAmp(driveTrain)); unused becuase we dont pickup from amp with a path anymore
    new Trigger(AimWhileMovingSup).whileTrue(new AimRobotMoving(
      driveTrain,
      () -> modifyAxis(-driverController.getRawAxis(Z_AXIS), DEADBAND_NORMAL),
      () -> modifyAxis(-driverController.getRawAxis(Y_AXIS), DEADBAND_NORMAL),
      () -> modifyAxis(-driverController.getRawAxis(X_AXIS), DEADBAND_NORMAL),
      driverController::getL2Button,
      StageAngleSup,
      FarStageAngleSup,
      SubwooferAngleSup,
      OverStagePassSup,
      OppositeStageShotSup,
      falseSup
      ));
    // new Trigger(()->driverController.getL3Button()&&driverController.getR3Button()).onTrue(
    //   new SequentialCommandGroup(
    //     new InstantCommand(()->angleShooterSubsystem.setDesiredShooterAngle(22.5), angleShooterSubsystem),
    //     new ParallelRaceGroup(
    //       new AimRobotMoving(driveTrain, zeroSup, zeroSup, zeroSup, ()->true, falseSup, falseSup, falseSup, falseSup, falseSup, ()->true),
    //       new ShootNote(indexer, 0.65, 0.4, intake)
    //     )
    //   )
    // );

    if(Preferences.getBoolean("Detector Limelight", false)) {
      Command AutoGroundIntake = new SequentialCommandGroup(
        new GroundIntake(intake, indexer, driveTrain, angleShooterSubsystem),
        new InstantCommand(()->RobotState.getInstance().IsNoteHeld = true)
      );
      autoPickup = new ParallelRaceGroup(
        new SequentialCommandGroup(
          new InstantCommand(()->angleShooterSubsystem.setDesiredShooterAngle(30), angleShooterSubsystem),
          new GroundIntake(intake, indexer, driveTrain, angleShooterSubsystem),
          new InstantCommand(()->RobotState.getInstance().setNoteHeld(true))),
        new SequentialCommandGroup(
          new CollectNote(driveTrain, limelight),
          new DriveTimeCommand(-1, 0, 0, 1.5, driveTrain),
          new DriveTimeCommand(1, 0, 0, 0.5, driveTrain),
          new DriveTimeCommand(-1, 0, 0, 0.5, driveTrain),
          new WaitCommand(0.5)
          )).withTimeout(4);
      podiumAutoPickup = new ParallelRaceGroup(
          new SequentialCommandGroup(
            new GroundIntake(intake, indexer, driveTrain, angleShooterSubsystem),
            new InstantCommand(()->RobotState.getInstance().setNoteHeld(true))),
          new SequentialCommandGroup(
            new PodiumCollectNote(driveTrain, limelight),
            new DriveTimeCommand(-1, 0, 0, 0.5, driveTrain),
            new DriveTimeCommand(1, 0, 0, 0.5, driveTrain),
            new DriveTimeCommand(-1, 0, 0, 0.5, driveTrain),
            new MoveMeters(driveTrain, 0.5, 0, -1, 0),
            new WaitCommand(0.5)
          )).withTimeout(4);
      // new Trigger(driverController::getR3ButtonPressed).whileTrue(GroundIntake);
      new Trigger(AutoPickupSup).whileTrue(autoPickup);
    }
    new Trigger(ForceVisionSup).onTrue(new InstantCommand(()->SmartDashboard.putBoolean("force use limelight", true))).onFalse(new InstantCommand(()->SmartDashboard.putBoolean("force use limelight", false)));
    SmartDashboard.putData("force update limelight position", new InstantCommand(()->driveTrain.forceUpdateOdometryWithVision(), driveTrain));
    if(angleShooterExists) {
      new Trigger(ShooterUpManualSup).whileTrue(new AngleShooter(angleShooterSubsystem, ()->ShooterConstants.PRAC_MAXIMUM_SHOOTER_ANGLE));
      SmartDashboard.putData("Manual Angle Shooter Up", new AngleShooter(angleShooterSubsystem, ()->ShooterConstants.PRAC_MAXIMUM_SHOOTER_ANGLE));
    }
    if(indexerExists) {
      new Trigger(ManualShootSup).whileTrue(new ManualShoot(indexer, driverController::getPOV, intake));
    }
    if(climberExists) {
    }
    if(shooterExists) {
    }
    if(intakeExists&&indexerExists) {
      new Trigger(GroundIntakeSup).whileTrue(new GroundIntake(intake, indexer, driveTrain, angleShooterSubsystem));
    }
    if(intakeExists&&indexerExists) {
      new Trigger(()->RobotState.getInstance().isNoteSeen()).and(()->!RobotState.getInstance().IsNoteHeld).and(()->DriverStation.isTeleop()).and(()->!AimWhileMovingSup.getAsBoolean()).onTrue(new IndexerNoteAlign(indexer, intake).withInterruptBehavior(InterruptionBehavior.kCancelIncoming).withTimeout(5));
    }
    
    SmartDashboard.putData("move 1 meter", new MoveMeters(driveTrain, 1, 0.2, 0.2, 0.2));
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
      SmartDashboard.putData("intake on",new SequentialCommandGroup(
        new InstantCommand(()->intake.intakeYes(IntakeConstants.INTAKE_SPEED, IntakeConstants.INTAKE_SIDE_SPEED), intake)
        // new InstantCommand(()->intake.intakeSideWheels(0.5), intake)));
      ));
      SmartDashboard.putData("intake off", new InstantCommand(intake::intakeOff, intake));
    }
    if(shooterExists) {
      SmartDashboard.putData("shooter on speaker", new InstantCommand(()->shooter.shootRPS(ShooterConstants.LONG_SHOOTING_RPS), shooter));
      SmartDashboard.putData("shooter on amp", new InstantCommand(()->shooter.shootRPS(ShooterConstants.PRAC_AMP_RPS), shooter));
      SmartDashboard.putNumber("run shooter speed", ShooterConstants.LONG_SHOOTING_RPS);
      SmartDashboard.putData("shooterSubsystem", shooter);
      SmartDashboard.putData("run shooter", new OverrideCommand(shooter) {
        @Override public void execute() { shooter.shootRPS(SmartDashboard.getNumber("run shooter speed", ShooterConstants.LONG_SHOOTING_RPS)); }
        @Override public void end(boolean interrupted) { shooter.turnOff(); }
      });
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
      SmartDashboard.putData("indexer shooting speed", new InstantCommand(()->indexer.set(IndexerConstants.INDEXER_SHOOTING_POWER)));
      SmartDashboard.putData("indexer off", new InstantCommand(()->indexer.off()));
    }
    if(climberExists) {
      SmartDashboard.putData("climber up", new InstantCommand(()->{
      climber.climberGoLeft(ClimberConstants.CLIMBER_SPEED_UP);
      climber.climberGoRight(ClimberConstants.CLIMBER_SPEED_UP);}));
      SmartDashboard.putData("climber down", new InstantCommand(()->{
      climber.climberGoLeft(ClimberConstants.CLIMBER_SPEED_DOWN);
      climber.climberGoRight(ClimberConstants.CLIMBER_SPEED_DOWN);}));
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

  public void autonomousInit() {
    SmartDashboard.putNumber("autos ran", SmartDashboard.getNumber("autos ran", 0)+1);
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

  private Pose2d getAmpShotPose(Alliance currentAlliance) {
    if(currentAlliance == null) { return new Pose2d(5,5, new Rotation2d(Math.toRadians(-90)));}
    else {
      if(currentAlliance == Alliance.Blue) {
        return new Pose2d(1.83, 6.88, new Rotation2d(Math.toRadians(-90)));
      } else {
        return new Pose2d(14.75, 6.88, new Rotation2d(Math.toRadians(-90)));
      }
    }
  } 
  private void registerNamedCommands() {
    NamedCommands.registerCommand("awayFromPodium", new MoveMeters(driveTrain, 0.2, 1, 0, 0));
    NamedCommands.registerCommand("stopDrivetrain", new InstantCommand(driveTrain::stop, driveTrain));
    NamedCommands.registerCommand("hardStop", new InstantCommand(driveTrain::pointWheelsInward, driveTrain));
    if(intakeExists&&indexerExists) {
    NamedCommands.registerCommand("driveBackwardsToIntake", new ParallelRaceGroup(
      new SequentialCommandGroup(
        new MoveMeters(driveTrain, 0.7, -2, 0, 0),
        new MoveMeters(driveTrain, 0.7, 2, 0, 0)),
      new AutoGroundIntake(indexer, intake, driveTrain)
    ));
    }
    if(autoPickup != null) {
      NamedCommands.registerCommand("autoPickup", autoPickup);
      NamedCommands.registerCommand("podiumAutoPickup", podiumAutoPickup);
    }
    if(intakeExists&&!indexerExists&&!angleShooterExists) {
      NamedCommands.registerCommand("groundIntake", new InstantCommand(()->intake.intakeYes(IntakeConstants.INTAKE_SPEED, IntakeConstants.INTAKE_SIDE_SPEED)));
      NamedCommands.registerCommand("autoShootNote", new AimRobotMoving(driveTrain, zeroSup, zeroSup, zeroSup, ()->true, falseSup, falseSup, falseSup, falseSup, falseSup, falseSup).withTimeout(1));
      NamedCommands.registerCommand("autoPickup", new SequentialCommandGroup(
        new CollectNote(driveTrain, limelight),
        new DriveTimeCommand(-1, 0, 0, 1, driveTrain)
      ).withTimeout(3));
    }
    if(shooterExists) {
      NamedCommands.registerCommand("shooterOn", new InstantCommand(()->shooter.shootRPS(LONG_SHOOTING_RPS), shooter));
      SmartDashboard.putData("shooterOn", new InstantCommand(()->shooter.shootRPS(LONG_SHOOTING_RPS), shooter));
    }
    if(intakeExists&&indexerExists&&angleShooterExists) {
      NamedCommands.registerCommand("shootNote", new ShootNote(indexer, 0.2, 0, intake));
      NamedCommands.registerCommand("sourceSideLongShot", new SequentialCommandGroup(
        new InstantCommand(()->angleShooterSubsystem.setDesiredShooterAngle(22.5), angleShooterSubsystem),
        new ParallelRaceGroup(
          new AimRobotMoving(driveTrain, zeroSup, zeroSup, zeroSup, ()->true, falseSup, falseSup, falseSup, falseSup, falseSup, ()->true),
          new ShootNote(indexer, 0.5, 0.4, intake)
        )
      ));
      NamedCommands.registerCommand("midAmpSideShooterAngle", new OverrideCommand(angleShooterSubsystem) {
        public void execute() {
          angleShooterSubsystem.setDesiredShooterAngle(27);
        };
      });
      NamedCommands.registerCommand("lowAmpSideShooterAngle", new OverrideCommand(angleShooterSubsystem) {
        public void execute() {
          angleShooterSubsystem.setDesiredShooterAngle(29.5);
        };
      });
    }
    if(indexerExists) {
      // NamedCommands.registerCommand("feedShooter", new InstantCommand(()->indexer.set(IndexerConstants.INDEXER_SHOOTING_SPEED), indexer));
      // NamedCommands.registerCommand("stopFeedingShooter", new InstantCommand(indexer::off, indexer));
    }
    if(intakeExists) {
      NamedCommands.registerCommand("intakeOn", new InstantCommand(()-> intake.intakeYes(IntakeConstants.INTAKE_SPEED, IntakeConstants.INTAKE_SIDE_SPEED)));
      if(sideWheelsExists){
        NamedCommands.registerCommand("intakeSideWheels", new InstantCommand(()-> intake.intakeSideWheels(1)));
      }
      NamedCommands.registerCommand("note isn't held", new WaitUntil(()->!RobotState.getInstance().isNoteSeen()));
    }
    if(indexerExists&&shooterExists) {
      NamedCommands.registerCommand("initialShot", new InitialShot(shooter, indexer, 0.9, 0.1, angleShooterSubsystem));
      //the following command will both aim the robot at the speaker (with the AimRobotMoving), and shoot a note while aiming the shooter (with shootNote). As a race group, it ends
      //when either command finishes. the AimRobotMoving command will never finish, but the shootNote finishes when shootTime is reached.
      NamedCommands.registerCommand("autoShootNote", new ParallelRaceGroup(
        new AimRobotMoving(driveTrain, zeroSup, zeroSup, zeroSup, ()->true, falseSup, falseSup, falseSup, falseSup, falseSup, falseSup),
        new SequentialCommandGroup(
          new InstantCommand(()->angleShooterSubsystem.setDesiredShooterAngle(angleShooterSubsystem.calculateSpeakerAngle()), angleShooterSubsystem),
          new ShootNote(indexer, 1.5, 0.5,intake)
        )));
      // NamedCommands.registerCommand("setFeedTrue", new InstantCommand(()->SmartDashboard.putBoolean("feedMotor", true)));
      // NamedCommands.registerCommand("setFeedFalse", new InstantCommand(()->SmartDashboard.putBoolean("feedMotor", false)));
    }
    if(angleShooterExists) {
      //the same command that we use during teleop, but all the buttons that would aim the shooter anywhere other than the speaker are set to false.
      NamedCommands.registerCommand("autoAimAtSpeaker", new AimShooter(angleShooterSubsystem, ()->false, ()->false, ()->false, ()->false, ()->false, ()->false, ()->false, ()->false, ()->false));
      SmartDashboard.putData("autoAimAtSpeaker", new AimShooter(angleShooterSubsystem, ()->false, ()->false, ()->false, ()->false, ()->false, ()->false, ()->false, ()->false, ()->false));
    }
    if (indexerExists&&intakeExists) {
      NamedCommands.registerCommand("groundIntake", new AutoGroundIntake(indexer, intake, driveTrain));
      SmartDashboard.putData("groundIntake", new AutoGroundIntake(indexer, intake, driveTrain));
    }
    NamedCommands.registerCommand("wait x seconds", new WaitCommand(Preferences.getDouble("wait # of seconds", 0)));
  }
  public void ampShotInit() {
    if(indexerExists&&shooterExists&&angleShooterExists) {
      double indexerAmpSpeed;
      double shooterAmpSpeed;
      if(Preferences.getBoolean("CompBot", true)) {
        shooterAmpSpeed = ShooterConstants.COMP_AMP_RPS;
        indexerAmpSpeed = IndexerConstants.COMP_INDEXER_AMP_SPEED;
      } else {
        shooterAmpSpeed = ShooterConstants.PRAC_AMP_RPS;
        indexerAmpSpeed = IndexerConstants.PRAC_INDEXER_AMP_SPEED;
      }
      SequentialCommandGroup scoreAmp = new SequentialCommandGroup(
        // new InstantCommand(()->shooter.shootSameRPS(ShooterConstants.AMP_RPS), shooter),
        new InstantCommand(()->shooter.shootWithSupplier(()->shooterAmpSpeed, true), shooter),
        new MoveMeters(driveTrain, 0.06, 0.3, 0, 0),
        // new WaitCommand(2),
        new WaitUntil(()->(shooter.validShot() && driveTrain.getChassisSpeeds().vxMetersPerSecond == 0)),
        new InstantCommand(()->indexer.magicRPS(indexerAmpSpeed), indexer),//45 worked but a bit too high
        new WaitCommand(0.5),
        new InstantCommand(()->RobotState.getInstance().setNoteHeld(false))
        );
        SmartDashboard.putNumber("Indexer Amp Speed", indexerAmpSpeed);
        /**
         * the following code producs a command that will first pathfind to a pose right in front of the amplifier, then drive backwards into the amp, then run our shooters amp shot 
         * sequence as it was at the 2024 State competition.
         */
      SequentialCommandGroup orbitAmpShot = new SequentialCommandGroup(
        new InstantCommand(()->shooter.setTargetVelocity(shooterAmpSpeed, shooterAmpSpeed, 50, 50), shooter),
        new InstantCommand(()->angleShooterSubsystem.setDesiredShooterAngle(50), angleShooterSubsystem),
        // new AutoBuilder().pathfindThenFollowPath(PathPlannerPath.fromPathFile("AmpShotSetup"), DEFAUL_PATH_CONSTRAINTS),
        Commands.select(Map.of(
          Alliance.Red, AutoBuilder.pathfindToPose(getAmpShotPose(Alliance.Red), DEFAULT_PATH_CONSTRAINTS),
          Alliance.Blue, AutoBuilder.pathfindToPose(getAmpShotPose(Alliance.Blue), DEFAULT_PATH_CONSTRAINTS)
        ), ()->currentAlliance==null ? Alliance.Red : currentAlliance),
        new MoveMeters(driveTrain, 0.9, -0.5, 0, 0),
        new MoveMeters(driveTrain, 0.015, 0.5, 0, 0),
        new InstantCommand(driveTrain::pointWheelsInward, driveTrain),
        new WaitUntil(()->(Math.abs(shooter.getLSpeed()-shooterAmpSpeed)<0.2)&&(Math.abs(shooter.getRSpeed()-shooterAmpSpeed)<0.3)),
        new InstantCommand(()->angleShooterSubsystem.setDesiredShooterAngle(Field.AMPLIFIER_SHOOTER_ANGLE)),
        new InstantCommand(()->indexer.magicRPSSupplier(()->indexerAmpSpeed), indexer),
        new WaitCommand(0.5),
        new InstantCommand(()->RobotState.getInstance().setNoteHeld(false))
      );
        new Trigger(AmpAngleSup).whileTrue(orbitAmpShot);
        SmartDashboard.putData("amp shot", scoreAmp);
    }
  }
  public void teleopInit() {
    if(climberExists) {
      SequentialCommandGroup resetClimbers = new SequentialCommandGroup(
        new InstantCommand(()->{climber.climberGoLeft(ClimberConstants.CLIMBER_SPEED_DOWN);climber.climberGoRight(ClimberConstants.CLIMBER_SPEED_DOWN);}, climber),
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
    } else {
      RobotState.getInstance().IsNoteSeen = false;
    }
    SmartDashboard.putBoolean("is note seen", RobotState.getInstance().IsNoteSeen);
		SmartDashboard.putBoolean("shooter in range", RobotState.getInstance().ShooterInRange);
  }
 
  public void logPower(){
    for(int i = 0; i < 16; i++) { 
      SmartDashboard.putNumber("PDP Current " + i, PDP.getCurrent(i));
    }
  }
  public void robotPeriodic() {
      currentAlliance = DriverStation.getAlliance().get();
      SmartDashboard.putBoolean("RobotPeriodicRan", true);
      SmartDashboard.putString("AlliancePeriodic", currentAlliance == null? "null" : currentAlliance == Alliance.Red? "Red": "Blue" );

      if(Preferences.getBoolean("Use Limelight", false)) {
        limelight.updateLoggingWithPoses();
      }

      SmartDashboard.putBoolean("is note in", RobotState.getInstance().isNoteSeen());
      SmartDashboard.putBoolean("is note held", RobotState.getInstance().IsNoteHeld);
    // logPower();
  }
  public void disabledPeriodic() {
  
  }

  public void disabledInit() {
    if(angleShooterExists) {
      new InstantCommand(angleShooterSubsystem::stop, angleShooterSubsystem);
    }
  }
}
