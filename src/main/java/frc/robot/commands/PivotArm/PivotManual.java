// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PivotArm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotArmSubsystem;

public class PivotManual extends CommandBase {
  /** Creates a new PivotManual. */
  XboxController xboxController;
  PivotArmSubsystem pivotArmSubsystem;

  public PivotManual(PivotArmSubsystem pivot, XboxController controller) {
    pivotArmSubsystem = pivot;
    xboxController = controller;
    addRequirements(pivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(xboxController.getRightY())>0.1){
      
      if (pivotArmSubsystem.getEnablePIDStage()){
        pivotArmSubsystem.toggleEnablePIDStage(false);
        pivotArmSubsystem.resetTargetPosition();
      }

      double currentTargetPosition = pivotArmSubsystem.getTargetPosition();
      currentTargetPosition += xboxController.getLeftY()*(15) ;
      pivotArmSubsystem.setTargetPosition(currentTargetPosition);

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
