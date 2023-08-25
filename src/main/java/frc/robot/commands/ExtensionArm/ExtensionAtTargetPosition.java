// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ExtensionArm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtensionArmSubsystem;

public class ExtensionAtTargetPosition extends CommandBase {
  /** Creates a new ExtensionAtTargetPosition. */
  ExtensionArmSubsystem extensionArmSubsystem;
  double atPosition;
  boolean resetPosition;
  boolean isFinished;
  double dir;

  public ExtensionAtTargetPosition(ExtensionArmSubsystem extension,  double pos, boolean reset) {
    extensionArmSubsystem = extension;
    atPosition = pos;
    resetPosition = reset;
    isFinished = false;
    dir = 0;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;

    // if (extensionArmSubsystem.getCurrentPosition() >= atPosition){
    //   dir = -1;
    // }
    // else{
    //   dir = 1;
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Math.abs(extensionArmSubsystem.getCurrentPosition()-atPosition) <=2.5){
      isFinished = true;
      if (resetPosition){
        extensionArmSubsystem.setCurrentPosition(0);
      }
    }

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
