// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.cert.TrustAnchor;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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

public class ExtensionArmSubsystem extends SubsystemBase {

  ShuffleboardTab extensionTab = Shuffleboard.getTab("extension");
  GenericEntry SBCurrentPosition = extensionTab.add("Current Position", 0).withPosition(0, 0).getEntry();
  GenericEntry SBTargettedPosition = extensionTab.add("Targetted Position", 0).withPosition(1, 0).getEntry();
  GenericEntry SBTargettedStage = extensionTab.add("Targetted Stage", 0).withPosition(2, 0).getEntry();
  GenericEntry SBTargettedPositionIn = extensionTab.add("Targetted Position In", 0).withPosition(2, 1).getEntry();
  GenericEntry SBTargettedStageIn = extensionTab.add("Targetted Stage In", 0).withPosition(3, 1).getEntry();

  GenericEntry SBEnablePID = extensionTab.add("Enable PID", 0).withPosition(0, 1).getEntry();
  GenericEntry SBEnablePIDStage = extensionTab.add("Enable Stage", false).withPosition(1, 1).getEntry();
  
  GenericEntry SBPIDP = extensionTab.add("PID P", 0).withPosition(4, 0).getEntry();
  GenericEntry SBPIDI = extensionTab.add("PID I", 0).withPosition(4, 1).getEntry();
  GenericEntry SBPIDD = extensionTab.add("PID D", 0).withPosition(4, 2).getEntry();
  GenericEntry SBResetPID = extensionTab.add("Reset PID", 0).withPosition(5, 0).getEntry();
  GenericEntry SBResetPosition = extensionTab.add("Reset Position", 0).withPosition(5, 1).getEntry();

  CANSparkMax m_extensionMotor;
  PIDController extensionPIDController;
  boolean enablePIDControllerStage;
  boolean enablePIDController;
  double targetPosition;
  double targetSetpoint;
  boolean isTroubleshoot;


  /** Creates a new ExtensionArmSubsystem. */
  public ExtensionArmSubsystem() {
    isTroubleshoot = Robotmap.troubleshooting;
    enablePIDController = true;
    enablePIDControllerStage = false;

    m_extensionMotor = new CANSparkMax( Robotmap.Extension.extensionMotorID,MotorType.kBrushless);
    m_extensionMotor.setInverted(false);   
    m_extensionMotor.getEncoder().setPosition(0);
    
    extensionPIDController = new PIDController(Robotmap.Extension.PIDP, Robotmap.Extension.PIDI,Robotmap.Extension.PIDD);
  
    targetPosition = 0;
    targetSetpoint =0;


    
  }

 
  public void runextensionSpeed(double speed){
    m_extensionMotor.set(speed);
  }

  public void setPID(double p, double i, double d){
    extensionPIDController.setP(p);
    extensionPIDController.setI(i);
    extensionPIDController.setD(d);
  }

  public void setReferencePosition(double Position){
    double calculate = extensionPIDController.calculate(getCurrentPosition(),Position);
    runextensionSpeed(calculate);
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
    else if (targetPosition < -61200 ) {
      targetPosition = -61200;
    }
  }

  public double getTargetPosition(){
    return targetPosition;
  }

  public double getCurrentPosition(){
    return m_extensionMotor.getEncoder().getPosition();
  }

  public void setCurrentPosition(double pos){
    m_extensionMotor.getEncoder().setPosition(pos);
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
    
    // if (getEnablePIDStage()){
    //   runStage();      
    // }

    if (getEnablePID()){
      setReferencePosition(targetPosition);
    }

    if (SBResetPosition.getDouble(0)==1){
      setCurrentPosition(0);
    }

    if (SBResetPID.getDouble(0)==1){
      double p = SBPIDP.getDouble(0);
      double i = SBPIDI.getDouble(0);
      double d = SBPIDD.getDouble(0);

      setPID(p, i, d);
    }

    SBCurrentPosition.setDouble(getCurrentPosition());
    SBTargettedPosition.setDouble(getTargetPosition());

 

  //  SBEnablePID.setBoolean(enablePIDController);
  //  SBEnablePIDStage.setBoolean(enablePIDControllerStage);
    
  //enablePIDController = (SBEnablePID.getDouble(0) ==1);
   
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
