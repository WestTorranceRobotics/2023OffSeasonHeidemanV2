// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robotmap;

public class IntakeSubsystem extends SubsystemBase {

  ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
  GenericEntry SBIntakeVelocity = intakeTab.add("Intake Velocity", 0).withPosition(0, 0).getEntry();
  GenericEntry SBIntakeStalling = intakeTab.add("Intake Stalling", false).withPosition(1, 0).getEntry();
  GenericEntry SBIntakeTargetSpeed = intakeTab.add("Intake Target Speed", 0).withPosition(2, 0).getEntry();

  /** Creates a new IntakeSubsystem. */
  CANSparkMax m_intakeMotor;
  double intakeSpeed;
  boolean coneCube;

  public IntakeSubsystem() {
    m_intakeMotor = new CANSparkMax( Robotmap.Intake.intakeMotorID , MotorType.kBrushless);
    m_intakeMotor.restoreFactoryDefaults();
    m_intakeMotor.setInverted(false);
    intakeSpeed = 0;
    coneCube = true;
  }


  public void runIntake(double speed){
    m_intakeMotor.set(-speed);
  }

  public double getIntakeSpeed(){
    return m_intakeMotor.getEncoder().getVelocity();
  }

  public double getTargettedIntakeSpeed(){
    return intakeSpeed;
  }
  public void setConeCube(boolean bol){
    coneCube = bol;
  }
  public boolean getConeCube(){
    return coneCube;
  }

  @Override
  public void periodic() {
    intakeSpeed = SBIntakeTargetSpeed.getDouble(0);
    SBIntakeVelocity.setDouble(m_intakeMotor.getEncoder().getVelocity());
    SBIntakeStalling.setBoolean(Math.abs(m_intakeMotor.getEncoder().getVelocity()) < 0);
    // This method will be called once per scheduler run
  }
}
