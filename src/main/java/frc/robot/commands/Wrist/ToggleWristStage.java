// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class ToggleWristStage extends CommandBase {
  /** Creates a new ToggleWristStage. */
  WristSubsystem wristSubsystem;
  double targetStage;
  boolean isFinished;
  public ToggleWristStage(WristSubsystem wrist, double setpoint) {
    targetStage = setpoint;
    wristSubsystem = wrist;
    isFinished = false;
    addRequirements(wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wristSubsystem.setStage(targetStage);
    wristSubsystem.toggleEnablePIDStage(true);
    isFinished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
