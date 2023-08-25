// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroup;

import java.sql.Time;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ExtensionArm.ToggleExtensionTargetPosition;
import frc.robot.commands.Intake.IntakeCustomCommand;
import frc.robot.commands.Intake.OuttakeCommand;
import frc.robot.commands.MiscCommands.Delay;
import frc.robot.commands.PivotArm.PivotAtTargetPosition;
import frc.robot.commands.PivotArm.TogglePivotTargetPosition;
import frc.robot.commands.Wrist.ToggleWristTargetPosition;
import frc.robot.commands.Wrist.WristAtTarget;
import frc.robot.subsystems.ExtensionArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreLow extends SequentialCommandGroup {
  /** Creates a new ScoreLow. */
 

  
  public ScoreLow( PivotArmSubsystem pivotArmSubsystem,
  ExtensionArmSubsystem extensionArmSubsystem,
  WristSubsystem wristSubsystem,
  IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelDeadlineGroup(new PivotAtTargetPosition(pivotArmSubsystem, -132, false), new TogglePivotTargetPosition(pivotArmSubsystem, -132))
    , new ParallelDeadlineGroup(new WristAtTarget(wristSubsystem, -27, false),new ToggleWristTargetPosition(wristSubsystem, -27))
    , new ParallelDeadlineGroup(new Delay(0.5),new IntakeCustomCommand(intakeSubsystem, -0.4) ));
  }
}
