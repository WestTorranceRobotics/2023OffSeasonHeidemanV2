// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robotmap;

public class WristSubsystem extends SubsystemBase {

  ShuffleboardTab wristTab = Shuffleboard.getTab("Wrist");
  GenericEntry SBCurrentPosition = wristTab.add("Current Position", 0).withPosition(0, 0).getEntry();
  GenericEntry SBTargettedPosition = wristTab.add("Targetted Position", 0).withPosition(1, 0).getEntry();
  GenericEntry SBTargettedStage = wristTab.add("Targetted Stage", 0).withPosition(2, 0).getEntry();
  GenericEntry SBTargettedPositionIn = wristTab.add("Targetted Position In", 0).withPosition(2, 1).getEntry();
  GenericEntry SBTargettedStageIn = wristTab.add("Targetted Stage In", 0).withPosition(3, 1).getEntry();

  GenericEntry SBEnablePID = wristTab.add("Enable PID", 0).withPosition(0, 1).getEntry();
  GenericEntry SBEnablePIDStage = wristTab.add("Enable Stage", false).withPosition(1, 1).getEntry();
  
  GenericEntry SBPIDP = wristTab.add("PID P", 0).withPosition(4, 0).getEntry();
  GenericEntry SBPIDI = wristTab.add("PID I", 0).withPosition(4, 1).getEntry();
  GenericEntry SBPIDD = wristTab.add("PID D", 0).withPosition(4, 2).getEntry();
  GenericEntry SBResetPID = wristTab.add("Reset PID", 0).withPosition(5, 0).getEntry();
  GenericEntry SBResetCurrentPosition = wristTab.add("Reset Position", 0).withPosition(5, 1).getEntry();





  /** Creates a new WristSubsystem. */
  CANSparkMax m_wristMotor;
  PIDController wristPIDController;
  boolean enablePIDControllerStage;
  boolean enablePIDController;
  double targetPosition;
  double targetSetpoint;
  boolean isTroubleshoot;


  public WristSubsystem() {
    isTroubleshoot = Robotmap.troubleshooting;
    enablePIDController = true;
    enablePIDControllerStage = false;

    m_wristMotor = new CANSparkMax(Robotmap.Wrists.wristMotorID, MotorType.kBrushless);
    m_wristMotor.restoreFactoryDefaults();
    m_wristMotor.setInverted(false);   
    m_wristMotor.getEncoder().setPosition(0); 
    
    wristPIDController = new PIDController(0, 0, 0);
    wristPIDController.setP(Robotmap.Wrists.PIDP);
    wristPIDController.setI(Robotmap.Wrists.PIDI);
    wristPIDController.setD(Robotmap.Wrists.PIDD);
  
    targetPosition = 0;
    targetSetpoint =0;


  }

  public void runWristSpeed(double speed){
    m_wristMotor.set(speed);
  }

  public void setPID(double p, double i, double d){
    wristPIDController.setP(p);
    wristPIDController.setI(i);
    wristPIDController.setD(d);
  }

  public void setReferencePosition(double Position){
    double speed = wristPIDController.calculate(getCurrentPosition(),Position);
    runWristSpeed(speed);
  }

  public void toggleEnablePIDStage(boolean bol){
    enablePIDControllerStage = bol;
  }

  public void toggleEnablePID(boolean bol){
    enablePIDController = bol;
  }



  public void setTargetPosition(double position){
    targetPosition = position;

    if (targetPosition > 0){
      targetPosition = 0;
    }
    else if (targetPosition <-200 ) {
      targetPosition = -200;
    }
  }

  public double getTargetPosition(){
    return targetPosition;
  }

  public double getCurrentPosition(){
    return m_wristMotor.getEncoder().getPosition();
  }

  public void setCurrentPosition(double pos){
    m_wristMotor.getEncoder().setPosition(pos);
  }
  public void resetTargetPosition(){
    targetPosition = getCurrentPosition();
  }

  public boolean getEnablePIDStage(){
    return enablePIDControllerStage;
  }

  public double getPIDStage(){
    return targetSetpoint;
  }

  public void setStage(double stage){
    targetSetpoint = stage;
  }

  
  public boolean getEnablePID(){
    return enablePIDController;
  }

  public void runStage(){
    if (targetSetpoint == 0){
      setTargetPosition(0);
    }

    else if (targetSetpoint == 1){
      setTargetPosition(-87);
    }

    else if (targetSetpoint == 2){
      setTargetPosition(-110);
    }
  }

  

  @Override
  public void periodic() {
    
    if (getEnablePIDStage()){
      runStage();      
    }

    if (enablePIDController){
      setReferencePosition(targetPosition);
    }

    if (SBResetPID.getDouble(0)==1){
      double p = SBPIDP.getDouble(0);
      double i = SBPIDI.getDouble(0);
      double d = SBPIDD.getDouble(0);

      setPID(p, i, d);
    }

    if (SBResetCurrentPosition.getDouble(0)==1){
      setCurrentPosition(0);
    }

    // SBEnablePID.setBoolean(enablePIDController);
    // SBEnablePIDStage.setBoolean(enablePIDControllerStage);

    //enablePIDControllerStage =  (SBEnablePIDStage.getDouble(0)==1);

    SBCurrentPosition.setDouble(getCurrentPosition());
    
    SBTargettedPosition.setDouble(targetPosition);
    SBTargettedStage.setDouble(targetSetpoint);

    if (isTroubleshoot){
      enablePIDController = (SBEnablePID.getDouble(0)==1);


      double targetPos = SBTargettedPositionIn.getDouble(0);
      setTargetPosition(targetPos);

      double targetStag = SBTargettedStageIn.getDouble(0);
      setStage(targetStag);

    }

    else{
      if (enablePIDController){
        SBEnablePID.setDouble(1);
      }
      else{
        SBEnablePID.setDouble(0);
      }
    }


    // This method will be called once per scheduler run
  }
}
