// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robotmap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class PivotArmSubsystem extends SubsystemBase {

  
  ShuffleboardTab pivotTab = Shuffleboard.getTab("Pivot");
  GenericEntry SBCurrentPosition = pivotTab.add("Current Position", 0).withPosition(0, 0).getEntry();
  GenericEntry SBTargettedPosition = pivotTab.add("Targetted Position", 0).withPosition(1, 0).getEntry();
  GenericEntry SBTargettedStage = pivotTab.add("Targetted Stage", 0).withPosition(2, 0).getEntry();
  GenericEntry SBTargettedPositionIn = pivotTab.add("Targetted Position In", 0).withPosition(2, 1).getEntry();
  GenericEntry SBTargettedStageIn = pivotTab.add("Targetted Stage In", 0).withPosition(3, 1).getEntry();

  GenericEntry SBEnablePID = pivotTab.add("Enable PID", 0).withPosition(0, 1).getEntry();
  GenericEntry SBEnablePIDStage = pivotTab.add("Enable Stage", false).withPosition(1, 1).getEntry();
  
  GenericEntry SBPIDP = pivotTab.add("PID P", 0).withPosition(4, 0).getEntry();
  GenericEntry SBPIDI = pivotTab.add("PID I", 0).withPosition(4, 1).getEntry();
  GenericEntry SBPIDD = pivotTab.add("PID D", 0).withPosition(4, 2).getEntry();
  GenericEntry SBResetPID = pivotTab.add("Reset PID", 0).withPosition(5, 0).getEntry();
  GenericEntry SBResetPosition = pivotTab.add("Reset Position", 0).withPosition(5, 1).getEntry();





  /** Creates a new pivotSubsystem. */
  WPI_TalonSRX m_pivotMotor;
  PIDController pivotPIDController;
  boolean enablePIDControllerStage;
  boolean enablePIDController;
  double targetPosition;
  double targetSetpoint;
  boolean isTroubleshoot;

  /** Creates a new PivotArmSubsystem. */
  public PivotArmSubsystem() {
    isTroubleshoot = Robotmap.troubleshooting;

    m_pivotMotor = new WPI_TalonSRX(Robotmap.Pivot.pivotMotorID);
    m_pivotMotor.setInverted(false);   
    m_pivotMotor.setSelectedSensorPosition(0); 
    
    pivotPIDController = new PIDController(0, 0, 0);
    pivotPIDController.setP(Robotmap.Pivot.PIDP);
    pivotPIDController.setI(Robotmap.Pivot.PIDI);
    pivotPIDController.setD(Robotmap.Pivot.PIDD);
  
    targetPosition = 0;
    targetSetpoint =0;
    enablePIDController = true;
    enablePIDControllerStage = false;

  }


  public void runpivotSpeed(double speed){
    m_pivotMotor.set(speed);
  }

  public void setPID(double p, double i, double d){
    pivotPIDController.setP(p);
    pivotPIDController.setI(i);
    pivotPIDController.setD(d);
  }

  public void setReferencePosition(double Position){
    double calculate = pivotPIDController.calculate(getCurrentPosition(), Position);
    runpivotSpeed(calculate);
  }

  public void toggleEnablePIDStage(boolean bol){
    enablePIDControllerStage = bol;
  }

  public void toggleEnablePID(boolean bol){
    enablePIDController = bol;
  }



  public void setTargetPosition(double in){
    targetPosition = in;

    if (targetPosition > 0){
      targetPosition = 0;
    }
    else if (targetPosition <-275 ) {
      targetPosition = -275;
    }
  }

  public double getTargetPosition(){
    return targetPosition;
  }

  public double getCurrentPosition(){
    return m_pivotMotor.getSelectedSensorPosition()/1000;
  }

  public void setCurrentPosition(double pos){
    m_pivotMotor.setSelectedSensorPosition(pos);
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
      setTargetPosition(0);
    }

    else if (targetSetpoint == 1){
      setTargetPosition(0);
    }
  }

  

  @Override
  public void periodic() {
    
    if (getEnablePIDStage()){
      runStage();      
    }

    if (getEnablePID()){
      setReferencePosition(targetPosition);
    }

    if (SBResetPID.getDouble(0)==1){
      double p = SBPIDP.getDouble(0);
      double i = SBPIDI.getDouble(0);                    
      double d = SBPIDD.getDouble(0);

      setPID(p, i, d);
    }
    if (SBResetPosition.getDouble(0)==1){
      setCurrentPosition(0);
    }

    // SBEnablePID.setBoolean(enablePIDController);
    // SBEnablePIDStage.setBoolean(enablePIDControllerStage);

   

   // enablePIDController = (SBEnablePID.getDouble(0) ==1);

    SBCurrentPosition.setDouble(getCurrentPosition());

    SBTargettedPosition.setDouble(getTargetPosition());

    if (isTroubleshoot){

      enablePIDController = (SBEnablePID.getDouble(0) ==1);
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
