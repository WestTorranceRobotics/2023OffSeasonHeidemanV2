// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.CommandGroup.GroundCubeIntake;
import frc.robot.commands.CommandGroup.GroundLyingConeIntake;
import frc.robot.commands.CommandGroup.HumanPlayerStationCone;
import frc.robot.commands.CommandGroup.HumanPlayerStationCube;
import frc.robot.commands.CommandGroup.ScoreHighCone;
import frc.robot.commands.CommandGroup.ScoreLow;
import frc.robot.commands.CommandGroup.ScoreMiddleCone;
import frc.robot.commands.CommandGroup.StartingPosition;
import frc.robot.commands.DriveTrain.DriveContinuous;
import frc.robot.commands.ExtensionArm.ExtensionSpeedManual;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Intake.IntakeToggleCubeCone;
import frc.robot.commands.Intake.OuttakeCommand;
import frc.robot.commands.PivotArm.PivotSpeedManual;
import frc.robot.commands.Wrist.WristSpeedManual;
import frc.robot.subsystems.ExtensionArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotArmSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.SwerveDriveTrain.DriveTrain;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  DriveTrain driveTrain;
  DriveContinuous driveContinuous;
  WristSubsystem wristSubsystem;
  WristSpeedManual wristSpeedManualUp;
  WristSpeedManual wristSpeedManualDown;

  IntakeCommand intakeCommand;
  OuttakeCommand outtakeCommand;

  PivotSpeedManual pivotSpeedManualUp;
  PivotSpeedManual pivotSpeedManualDown;
  PivotArmSubsystem pivotArmSubsystem;


  ExtensionArmSubsystem extensionArmSubsystem;
  ExtensionSpeedManual extensionSpeedManualDown;
  ExtensionSpeedManual extensionSpeedManualUp;

  IntakeSubsystem intakeSubsystem;

  LEDs led;


  private  XboxController driverxboxController = new XboxController(0);
  private  JoystickButton driverXButton =new JoystickButton(driverxboxController, 3);
  private  JoystickButton driverBButton =new JoystickButton(driverxboxController, 2);
  private  JoystickButton driverYButton =new JoystickButton(driverxboxController, 4);
  private  JoystickButton driverAButton =new JoystickButton(driverxboxController, 1);
  private  JoystickButton driverStartButton =new JoystickButton(driverxboxController, 8);


  private POVButton driverPOVRight = new POVButton(driverxboxController, 90);
  private POVButton driverPOVDown = new POVButton(driverxboxController, 180);
  private POVButton driverPOVLeft = new POVButton(driverxboxController, 270);
  private POVButton driverPOVUp = new POVButton(driverxboxController, 0);

  private  JoystickButton driverRightBumperButton =new JoystickButton(driverxboxController, 6);
  private  JoystickButton driverLeftBumperButton =new JoystickButton(driverxboxController, 5);
  




  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    driveTrain = new frc.robot.subsystems.SwerveDriveTrain.DriveTrain();
    driveContinuous = new DriveContinuous(driveTrain, driverxboxController);
    wristSubsystem = new WristSubsystem();
    driveTrain.setDefaultCommand(driveContinuous);
    wristSpeedManualUp = new WristSpeedManual(wristSubsystem, 0.1);
    wristSpeedManualDown = new WristSpeedManual(wristSubsystem, -0.1);

    intakeSubsystem = new IntakeSubsystem();

    intakeCommand = new IntakeCommand(intakeSubsystem);
    outtakeCommand = new OuttakeCommand(intakeSubsystem);
    
    pivotArmSubsystem = new PivotArmSubsystem();
    pivotSpeedManualDown = new PivotSpeedManual(pivotArmSubsystem, -0.1);
    pivotSpeedManualUp = new PivotSpeedManual(pivotArmSubsystem, 0.1);

    extensionArmSubsystem = new ExtensionArmSubsystem();
    extensionSpeedManualDown = new ExtensionSpeedManual(extensionArmSubsystem, -0.1);
    extensionSpeedManualUp = new ExtensionSpeedManual(extensionArmSubsystem, 0.1);

    led = new LEDs(DriveTrain.DRIVE_GYRO);

    
  

    // Configure the trigger bindings
    configureBindings();
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
        // driverYButton.whileTrue(extensionSpeedManualUp);
        // driverAButton.whileTrue(extensionSpeedManualDown);

        // driverXButton.whileTrue(pivotSpeedManualUp);
        // driverBButton.whileTrue(pivotSpeedManualDown);

        // driverPOVUp.whileTrue(wristSpeedManualUp);
        // driverPOVDown.whileTrue(wristSpeedManualDown);
        driverPOVLeft.whileTrue(intakeCommand);
        driverPOVRight.whileTrue(outtakeCommand);


      driverBButton.onTrue(new ScoreLow(pivotArmSubsystem, extensionArmSubsystem, wristSubsystem, intakeSubsystem));
      driverYButton.onTrue(new ScoreMiddleCone(pivotArmSubsystem, extensionArmSubsystem, wristSubsystem, intakeSubsystem));
      driverXButton.onTrue(new ScoreHighCone(pivotArmSubsystem, extensionArmSubsystem, wristSubsystem, intakeSubsystem));
      driverAButton.onTrue(new StartingPosition(pivotArmSubsystem, extensionArmSubsystem, wristSubsystem));
      
      driverLeftBumperButton.onTrue(new GroundLyingConeIntake(pivotArmSubsystem, extensionArmSubsystem, wristSubsystem, intakeSubsystem));
      driverRightBumperButton.onTrue(new GroundCubeIntake(pivotArmSubsystem, extensionArmSubsystem, wristSubsystem, intakeSubsystem));

      driverPOVUp.onTrue(new HumanPlayerStationCone(pivotArmSubsystem, extensionArmSubsystem, wristSubsystem, intakeSubsystem));
      driverPOVDown.onTrue(new HumanPlayerStationCube(pivotArmSubsystem, extensionArmSubsystem, wristSubsystem, intakeSubsystem));
         driverStartButton.onTrue(new IntakeToggleCubeCone(intakeSubsystem));
    
    
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(){
     return null;
  //   // An example command will be run in autonomous
  //   {
  //     // 1. Create trajectory settings
  //     TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
  //             AutoConstants.kMaxSpeedMetersPerSecond,
  //             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
  //                     .setKinematics(DriveConstants.kDriveKinematics);

  //     // 2. Generate trajectory
  //     Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
  //             new Pose2d(0, 0, new Rotation2d(0)),
  //             List.of(
  //                     new Translation2d(1, 0),
  //                     new Translation2d(1, -1)),
  //             new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
  //             trajectoryConfig);

  //     // 3. Define PID controllers for tracking trajectory
  //     PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
  //     PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
  //     ProfiledPIDController thetaController = new ProfiledPIDController(
  //             AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
  //     thetaController.enableContinuousInput(-Math.PI, Math.PI);

  //     // 4. Construct command to follow trajectory
  //     SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
  //             trajectory,
  //             swerveSubsystem::getPose,
  //             DriveConstants.kDriveKinematics,
  //             xController,
  //             yController,
  //             thetaController,
  //             swerveSubsystem::setModuleStates,
  //             swerveSubsystem);

  //     // 5. Add some init and wrap-up, and return everything
  //     return new SequentialCommandGroup(
  //             new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
  //             swerveControllerCommand,
  //             new InstantCommand(() -> swerveSubsystem.stopModules()));
  // }
  }
}
