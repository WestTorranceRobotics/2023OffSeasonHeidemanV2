// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class WristManualControl extends CommandBase {
  /** Creates a new WristManualControl. */
  XboxController xboxController;
  WristSubsystem wristSubsystem;
  public WristManualControl(WristSubsystem wrist, XboxController controller) {
    xboxController = controller;
    wristSubsystem = wrist;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(xboxController.getRightY())>0.1){
      
      if (wristSubsystem.getEnablePIDStage()){
        wristSubsystem.toggleEnablePIDStage(false);
        wristSubsystem.resetTargetPosition();
      }

      double currentTargetPosition = wristSubsystem.getTargetPosition();
      currentTargetPosition += xboxController.getRightY()*(15) ;
      wristSubsystem.setTargetPosition(currentTargetPosition);

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
