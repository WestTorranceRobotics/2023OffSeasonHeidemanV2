// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ExtensionArm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtensionArmSubsystem;

public class ExtensionManual extends CommandBase {
  /** Creates a new ExtensionManual. */
  XboxController xboxController;
  ExtensionArmSubsystem extensionArmSubsystem;
  public ExtensionManual(ExtensionArmSubsystem extension, XboxController controller) {
    xboxController = controller;
    extensionArmSubsystem = extension;
    addRequirements(extension);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Math.abs(xboxController.getRightTriggerAxis())>0.1){
      
      if (extensionArmSubsystem.getEnablePIDStage()){
        extensionArmSubsystem.toggleEnablePIDStage(false);
        extensionArmSubsystem.resetTargetPosition();
      }

      double currentTargetPosition = extensionArmSubsystem.getTargetPosition();
      currentTargetPosition += xboxController.getRightTriggerAxis()*(15) ;
      extensionArmSubsystem.setTargetPosition(currentTargetPosition);


    }

    if (Math.abs(xboxController.getLeftTriggerAxis())>0.1){
      
      if (extensionArmSubsystem.getEnablePIDStage()){
        extensionArmSubsystem.toggleEnablePIDStage(false);
        extensionArmSubsystem.resetTargetPosition();
      }

      double currentTargetPosition = extensionArmSubsystem.getTargetPosition();
      currentTargetPosition -= xboxController.getLeftTriggerAxis()*(15) ;
      extensionArmSubsystem.setTargetPosition(currentTargetPosition);

    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
