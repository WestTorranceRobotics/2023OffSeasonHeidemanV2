// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveTrain.DriveTrain;

public class DriveContinuous extends CommandBase {
  XboxController xboxController;
  DriveTrain driveTrain;
  /** Creates a new DriveContinuous. */
  public DriveContinuous(DriveTrain dt, XboxController xController) {
    this.driveTrain = dt;
    this.xboxController = xController;
    addRequirements(dt);
    // Use addRequirements() here to declare subsystem dependencies.
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  


    double x = MathUtil.applyDeadband(-xboxController.getLeftX(), 0.01);
    double y = MathUtil.applyDeadband(-xboxController.getLeftY(), 0.01);
    double rotation = MathUtil.applyDeadband(xboxController.getRightX(), 0.01);
    x *= Math.abs(x);
    y *= Math.abs(y);
    rotation *= Math.abs(rotation);
    driveTrain.setSwerveDrive(x,y,rotation);


    // if  ((Math.hypot(x, y) >=0.005) || (Math.abs(rotation) >= 0.005) ){
    //   driveTrain.setSwerveDrive(x,y,rotation);
    // }
     double reportangle = Math.atan2(x, y)*(180/Math.PI);
     reportangle *= -1;
    // if (reportangle <=0){
    //   reportangle*=-1;
    // }
    // else{
    //   reportangle = 360-reportangle;
    // }



    driveTrain.setShuffleboardController(reportangle);

    // double testangle = Math.atan2(x, y);
    // double testmagnitude =  MathUtil.applyDeadband(Math.hypot(x, y), 0.05);
    // double testx = Math.sin(testangle) * testmagnitude;
    // double testy = Math.cos(testangle) * testmagnitude;
    // double testdirection = Math.atan2(testy, testx)*(180/Math.PI);
    // System.out.println("x Before: " + Math.round(y*100) + " x After: "+ Math.round(testy*100));

    // driveTrain.setShuffleBoardFinalAngle(testdirection);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setSwerveDrive(0,0,0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
