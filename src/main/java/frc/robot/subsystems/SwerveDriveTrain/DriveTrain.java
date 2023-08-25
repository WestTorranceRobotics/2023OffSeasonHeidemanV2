// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveDriveTrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Robotmap;


public class DriveTrain extends SubsystemBase {

  ShuffleboardTab drivesTab = Shuffleboard.getTab("DriveTab");

  private GenericEntry SBWheelRFPosition = drivesTab.add("RF Position", 0).withPosition(0, 0).getEntry();
  private GenericEntry SBWheelRFSpeed = drivesTab.add("RF Speed", 0).withPosition(1, 0).getEntry();
  private GenericEntry SBWheelRFTPosition = drivesTab.add("RF T Position", 0).withPosition(2, 0).getEntry();
  private GenericEntry SBWheelRFTSpeed = drivesTab.add("RF T Speed", 0).withPosition(3, 0).getEntry();
  private GenericEntry SBWheelRFTResetPosition = drivesTab.add("RF Reset Position", 0).withPosition(4, 0).getEntry();


  private GenericEntry SBWheelLFPosition = drivesTab.add("LF Position", 0).withPosition(0, 1).getEntry();
  private GenericEntry SBWheelLFSpeed = drivesTab.add("LF Speed", 0).withPosition(1, 1).getEntry();
  private GenericEntry SBWheelLFTPosition = drivesTab.add("LF T Position", 0).withPosition(2, 1).getEntry();
  private GenericEntry SBWheelLFTSpeed = drivesTab.add("LF T Speed", 0).withPosition(3, 1).getEntry();
  private GenericEntry SBWheelLFTResetPosition = drivesTab.add("LF Reset Position", 0).withPosition(4, 1).getEntry();


  private GenericEntry SBWheelRBPosition = drivesTab.add("RB Position", 0).withPosition(0, 2).getEntry();
  private GenericEntry SBWheelRBSpeed = drivesTab.add("RB Speed", 0).withPosition(1, 2).getEntry();
  private GenericEntry SBWheelRBTPosition = drivesTab.add("RB T Position", 0).withPosition(2, 2).getEntry();
  private GenericEntry SBWheelRBTSpeed = drivesTab.add("RB T Speed", 0).withPosition(3, 2).getEntry();
  private GenericEntry SBWheelRBTResetPosition = drivesTab.add("RB Reset Position", 0).withPosition(4, 2).getEntry();


  private GenericEntry SBWheelLBPosition = drivesTab.add("LB Position", 0).withPosition(0, 3).getEntry();
  private GenericEntry SBWheelLBSpeed = drivesTab.add("LB Speed", 0).withPosition(1, 3).getEntry();
  private GenericEntry SBWheelLBTPosition = drivesTab.add("LB T Position", 0).withPosition(2, 3).getEntry();
  private GenericEntry SBWheelLBTSpeed = drivesTab.add("LB T Speed", 0).withPosition(3, 3).getEntry();
  private GenericEntry SBWheelLBTResetPosition = drivesTab.add("LB Reset Position", 0).withPosition(4, 3).getEntry();


  private GenericEntry SBEnablePIDLF= drivesTab.add("Enable PID LF", 0).withPosition(0, 4).getEntry();
  private GenericEntry SBEnablePIDLB= drivesTab.add("Enable PID LB", 0).withPosition(1, 4).getEntry();
  private GenericEntry SBEnablePIDRF= drivesTab.add("Enable PID RF", 0).withPosition(2, 4).getEntry();
  private GenericEntry SBEnablePIDRB= drivesTab.add("Enable PID RB", 0).withPosition(3, 4).getEntry();
  private GenericEntry SBControllerInput= drivesTab.add("Controller Input", 0).withPosition(4, 4).getEntry();

  private GenericEntry SBSpeedPIDP = drivesTab.add("Speed PID P", 0).withPosition(6, 0).getEntry();
  private GenericEntry SBSpeedPIDI = drivesTab.add("Speed PID I", 0).withPosition(6, 1).getEntry();
  private GenericEntry SBSpeedPIDD = drivesTab.add("Speed PID D", 0).withPosition(6, 2).getEntry();
  private GenericEntry SBTargettedSpeed = drivesTab.add("Targetted Speed", 0).withPosition(7, 0).getEntry();
  private GenericEntry SBResetSpeedPID = drivesTab.add("Reset Speed PID", 0).withPosition(7, 1).getEntry();
  private GenericEntry SBGyroYaw = drivesTab.add("Gyro Yaw", 0).withPosition(7, 2).getEntry();
  private GenericEntry SBGyroRoll = drivesTab.add("Gyro Roll", 0).withPosition(7, 3).getEntry();
  private GenericEntry SBGyroPitch = drivesTab.add("Gyro Pitch", 0).withPosition(7, 4).getEntry();
  private GenericEntry SBDirectionFInal= drivesTab.add("Direction Final", 0).withPosition(6, 3).getEntry();
  private GenericEntry SBSpeedPercentage= drivesTab.add("Speed Percentage", 0.3).withPosition(6, 4).getEntry();



  private static WPI_TalonFX LEFT_FRONT_DRIVE_SPEED_MOTOR;
  private static WPI_TalonFX LEFT_BACK_DRIVE_SPEED_MOTOR;
  private static WPI_TalonFX RIGHT_FRONT_DRIVE_SPEED_MOTOR;
  private static WPI_TalonFX RIGHT_BACK_DRIVE_SPEED_MOTOR;

  private static CANSparkMax LEFT_FRONT_DRIVE_DIRECTION_MOTOR;
  private static CANSparkMax LEFT_BACK_DRIVE_DIRECTION_MOTOR;
  private static CANSparkMax RIGHT_FRONT_DRIVE_DIRECTION_MOTOR;
  private static CANSparkMax RIGHT_BACK_DRIVE_DIRECTION_MOTOR;

  public static Translation2d m_frontLeftLocationTranslation2d;
  public static Translation2d m_frontRightLocatioTranslation2d;
  public static Translation2d m_backLeftLocationTranslation2d;
  public static Translation2d m_backRightLocationTranslation2d;

  public static SwerveDriveKinematics swervedriveKinematic;
  private static SwerveDriveOdometry swerveDriveOdometry;

  
  

  double a_kP;
  double a_kI;
  double a_kD;

  // CANCoders

  public static PIDController DRIVE_DISTANCE_CANCoder;

  public static CANCoder LEFT_FRONT_DRIVE_DIRECTION_CANCoder;
  public static CANCoder LEFT_BACK_DRIVE_DIRECTION_CANCoder;
  public static CANCoder RIGHT_FRONT_DRIVE_DIRECTION_CANCoder;
  public static CANCoder RIGHT_BACK_DRIVE_DIRECTION_CANCoder;

  // Direction CANCoder wrapper that scales to degrees
  public static PIDController LEFT_FRONT_DRIVE_DIRECTION_CONTROLLER;
  public static PIDController LEFT_BACK_DRIVE_DIRECTION_CONTROLLER;
  public static PIDController RIGHT_FRONT_DRIVE_DIRECTION_CONTROLLER;
  public static PIDController RIGHT_BACK_DRIVE_DIRECTION_CONTROLLER;
  
  // Gyro
  public static AHRS DRIVE_GYRO;

  SwerveDriveWheel LEFT_FRONT_DRIVE_WHEEL;
  SwerveDriveWheel LEFT_BACK_DRIVE_WHEEL;
  SwerveDriveWheel RIGHT_FRONT_DRIVE_WHEEL;
  SwerveDriveWheel RIGHT_BACK_DRIVE_WHEEL;
  /** Creates a new SwerveDrive. */
  public SwerveDriveCoordinator SWERVE_DRIVE_COORDINATOR;

  PIDController Swerve_Test_PIDCONTROLLER;

  //CANSparkMax testdumb;
  

  public DriveTrain() {
   // testdumb = new CANSparkMax(14, MotorType.kBrushless);
     // Motors
     LEFT_FRONT_DRIVE_SPEED_MOTOR = new WPI_TalonFX(Robotmap.SwerveDrive.LEFT_FRONT_DRIVE_SPEED_MOTOR_PIN);
     LEFT_BACK_DRIVE_SPEED_MOTOR = new WPI_TalonFX(Robotmap.SwerveDrive.LEFT_BACK_DRIVE_SPEED_MOTOR_PIN);
     RIGHT_FRONT_DRIVE_SPEED_MOTOR = new WPI_TalonFX(Robotmap.SwerveDrive.RIGHT_FRONT_DRIVE_SPEED_MOTOR_PIN);
     RIGHT_BACK_DRIVE_SPEED_MOTOR = new WPI_TalonFX(Robotmap.SwerveDrive.RIGHT_BACK_DRIVE_SPEED_MOTOR_PIN);

     LEFT_FRONT_DRIVE_SPEED_MOTOR.setNeutralMode(NeutralMode.Coast);
     RIGHT_FRONT_DRIVE_SPEED_MOTOR.setNeutralMode(NeutralMode.Coast);
     LEFT_BACK_DRIVE_SPEED_MOTOR.setNeutralMode(NeutralMode.Coast);
     RIGHT_BACK_DRIVE_SPEED_MOTOR.setNeutralMode(NeutralMode.Coast);


     LEFT_FRONT_DRIVE_DIRECTION_MOTOR = new CANSparkMax(Robotmap.SwerveDrive.LEFT_FRONT_DRIVE_DIRECTION_MOTOR_PIN, MotorType.kBrushless);
     LEFT_BACK_DRIVE_DIRECTION_MOTOR = new CANSparkMax(Robotmap.SwerveDrive.LEFT_BACK_DRIVE_DIRECTION_MOTOR_PIN,MotorType.kBrushless);
     RIGHT_FRONT_DRIVE_DIRECTION_MOTOR = new CANSparkMax(Robotmap.SwerveDrive.RIGHT_FRONT_DRIVE_DIRECTION_MOTOR_PIN,MotorType.kBrushless);
     RIGHT_BACK_DRIVE_DIRECTION_MOTOR = new CANSparkMax(Robotmap.SwerveDrive.RIGHT_BACK_DRIVE_DIRECTION_MOTOR_PIN,MotorType.kBrushless);

     LEFT_FRONT_DRIVE_DIRECTION_MOTOR.setIdleMode(IdleMode.kBrake);
     LEFT_BACK_DRIVE_DIRECTION_MOTOR.setIdleMode(IdleMode.kBrake);
     RIGHT_FRONT_DRIVE_DIRECTION_MOTOR.setIdleMode(IdleMode.kBrake);
     RIGHT_BACK_DRIVE_DIRECTION_MOTOR.setIdleMode(IdleMode.kBrake);

     // CANCoders
     //DRIVE_CANCoderS = new MedianPIDSource(LEFT_FRONT_DRIVE_DISTANCE_CANCoder, LEFT_BACK_DRIVE_DISTANCE_CANCoder, RIGHT_FRONT_DRIVE_DISTANCE_CANCoder, RIGHT_BACK_DRIVE_DISTANCE_CANCoder);

     LEFT_FRONT_DRIVE_DIRECTION_CANCoder = new CANCoder(Robotmap.SwerveDrive.LEFT_FRONT_DRIVE_DIRECTION_ENCODER_PIN);
     LEFT_BACK_DRIVE_DIRECTION_CANCoder = new CANCoder(Robotmap.SwerveDrive.LEFT_BACK_DRIVE_DIRECTION_ENCODER_PIN);
     RIGHT_FRONT_DRIVE_DIRECTION_CANCoder = new CANCoder(Robotmap.SwerveDrive.RIGHT_FRONT_DRIVE_DIRECTION_ENCODER_PIN);
     RIGHT_BACK_DRIVE_DIRECTION_CANCoder = new CANCoder(Robotmap.SwerveDrive.RIGHT_BACK_DRIVE_DIRECTION_ENCODER_PIN);

     LEFT_BACK_DRIVE_DIRECTION_CANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
     LEFT_FRONT_DRIVE_DIRECTION_CANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
     RIGHT_FRONT_DRIVE_DIRECTION_CANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
     RIGHT_BACK_DRIVE_DIRECTION_CANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
     
     // Direction CANCoder wrapper that scales to degrees
     LEFT_FRONT_DRIVE_DIRECTION_CONTROLLER = new PIDController(Robotmap.SwerveDrive.DRIVE_DIRECTION_P,Robotmap.SwerveDrive.DRIVE_DIRECTION_I,Robotmap.SwerveDrive.DRIVE_DIRECTION_D);
     LEFT_BACK_DRIVE_DIRECTION_CONTROLLER = new PIDController(Robotmap.SwerveDrive.DRIVE_DIRECTION_P,Robotmap.SwerveDrive.DRIVE_DIRECTION_I,Robotmap.SwerveDrive.DRIVE_DIRECTION_D);
     RIGHT_FRONT_DRIVE_DIRECTION_CONTROLLER = new PIDController(Robotmap.SwerveDrive.DRIVE_DIRECTION_P,Robotmap.SwerveDrive.DRIVE_DIRECTION_I,Robotmap.SwerveDrive.DRIVE_DIRECTION_D);
     RIGHT_BACK_DRIVE_DIRECTION_CONTROLLER = new PIDController(Robotmap.SwerveDrive.DRIVE_DIRECTION_P,Robotmap.SwerveDrive.DRIVE_DIRECTION_I,Robotmap.SwerveDrive.DRIVE_DIRECTION_D);

     // Gyro
     DRIVE_GYRO = new AHRS(Port.kMXP);
     DRIVE_GYRO.calibrate();
     DRIVE_GYRO.zeroYaw();
     DRIVE_GYRO.reset();
    

      // SwerveDriveWheels
      double wheelP = 0.015;
      double wheelI = 0.001;
      double wheelD = 0.0;
      LEFT_FRONT_DRIVE_WHEEL = new SwerveDriveWheel(wheelP, wheelI, wheelD, LEFT_FRONT_DRIVE_DIRECTION_CANCoder, LEFT_FRONT_DRIVE_DIRECTION_MOTOR, LEFT_FRONT_DRIVE_SPEED_MOTOR, Robotmap.SwerveDrive.LEFT_FRONT_DRIVE_DIRECTION_ENCODER_Offset);
      LEFT_BACK_DRIVE_WHEEL = new SwerveDriveWheel(wheelP, wheelI, wheelD, LEFT_BACK_DRIVE_DIRECTION_CANCoder, LEFT_BACK_DRIVE_DIRECTION_MOTOR, LEFT_BACK_DRIVE_SPEED_MOTOR, Robotmap.SwerveDrive.LEFT_BACK_DRIVE_DIRECTION_ENCODER_Offset);
      RIGHT_FRONT_DRIVE_WHEEL = new SwerveDriveWheel(wheelP, wheelI, wheelD, RIGHT_FRONT_DRIVE_DIRECTION_CANCoder, RIGHT_FRONT_DRIVE_DIRECTION_MOTOR, RIGHT_FRONT_DRIVE_SPEED_MOTOR, Robotmap.SwerveDrive.RIGHT_FRONT_DRIVE_DIRECTION_ENCODER_Offset);
      RIGHT_BACK_DRIVE_WHEEL = new SwerveDriveWheel(wheelP, wheelI, wheelD, RIGHT_BACK_DRIVE_DIRECTION_CANCoder, RIGHT_BACK_DRIVE_DIRECTION_MOTOR, RIGHT_BACK_DRIVE_SPEED_MOTOR,Robotmap.SwerveDrive.RIGHT_BACK_DRIVE_DIRECTION_ENCODER_Offset); 
      // SwerveDriveCoordinator
      SWERVE_DRIVE_COORDINATOR = new SwerveDriveCoordinator(DRIVE_GYRO, LEFT_FRONT_DRIVE_WHEEL, LEFT_BACK_DRIVE_WHEEL, RIGHT_FRONT_DRIVE_WHEEL, RIGHT_BACK_DRIVE_WHEEL);




      // // Locations for the swerve drive modules relative to the robot center.

      // m_frontLeftLocationTranslation2d = new Translation2d(0.381, 0.381);
      // m_frontRightLocatioTranslation2d = new Translation2d(0.381, -0.381);
      // m_backLeftLocationTranslation2d= new Translation2d(-0.381, 0.381);
      // m_backRightLocationTranslation2d = new Translation2d(-0.381, -0.381);

      // swervedriveKinematic =new SwerveDriveKinematics(
      //   m_frontLeftLocationTranslation2d, m_frontRightLocatioTranslation2d, m_backLeftLocationTranslation2d, m_backRightLocationTranslation2d);

      swerveDriveOdometry = new SwerveDriveOdometry(Robotmap.SwerveDrive.swervedriveKinematic, new Rotation2d(0),  new SwerveModulePosition[]{
        LEFT_FRONT_DRIVE_WHEEL.gModulePosition(),
        RIGHT_FRONT_DRIVE_WHEEL.gModulePosition(),
        LEFT_BACK_DRIVE_WHEEL.gModulePosition(),
        RIGHT_BACK_DRIVE_WHEEL.gModulePosition()});
        Swerve_Test_PIDCONTROLLER = new PIDController(wheelP, wheelI, wheelD);
  }

  public double getGyroRaw(){
    return DRIVE_GYRO.getYaw();

  }

  public void setSwerveDrive(double x, double y, double rotPower){
    SWERVE_DRIVE_COORDINATOR.setSwerveDrive(x, y, rotPower, SBSpeedPercentage.getDouble(0.3));
  }


  public void setShuffleboardController(double in){
    SBControllerInput.setDouble(in);
  }

  public void setShuffleboardTargettedSpeed(double in){
    SBTargettedSpeed.setDouble(in);
  }

  public void setShuffleBoardFinalAngle(double in){
    SBDirectionFInal.setDouble(in);
  }
  public Pose2d getPose2d(){
    return swerveDriveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
    swerveDriveOdometry.resetPosition(SWERVE_DRIVE_COORDINATOR.getRotation2d() , new SwerveModulePosition[]{
      LEFT_FRONT_DRIVE_WHEEL.gModulePosition(),
      RIGHT_FRONT_DRIVE_WHEEL.gModulePosition(),
      LEFT_BACK_DRIVE_WHEEL.gModulePosition(),
      RIGHT_BACK_DRIVE_WHEEL.gModulePosition()}, pose);
  }
  public static void resetGyro(){
    DRIVE_GYRO.calibrate();
    DRIVE_GYRO.zeroYaw();
    DRIVE_GYRO.reset();
  }

  

  /* 
      
  public void runWheelPosition(double Position){
    double calculate = MathUtil.clamp(Swerve_Test_PIDCONTROLLER.calculate(RIGHT_FRONT_DRIVE_DIRECTION_CANCoder.getPosition(), Position), -0.75, 0.75);
    RIGHT_FRONT_DRIVE_DIRECTION_MOTOR.set(-calculate);
  }

  public void runWheelSpeed(double speed){
    RIGHT_FRONT_DRIVE_DIRECTION_MOTOR.set(speed);
  }

  
*/

      

  @Override
  public void periodic() {
    //RIGHT_BACK_DRIVE_DIRECTION_MOTOR.set(0.6);

    //testdumb.set(0.1);
    
    // swerveDriveOdometry.update(SWERVE_DRIVE_COORDINATOR.getRotation2d(), new SwerveModulePosition[]{
    //   LEFT_FRONT_DRIVE_WHEEL.gModulePosition(),
    //   RIGHT_FRONT_DRIVE_WHEEL.gModulePosition(),
    //   LEFT_BACK_DRIVE_WHEEL.gModulePosition(),
    //   RIGHT_BACK_DRIVE_WHEEL.gModulePosition()});

    /*
    if ((a_kP != SBRotationKp.getDouble(0)) || (a_kI != SBRotationKi.getDouble(0)) || (a_kD != SBRotationKd.getDouble(0))){
      a_kP = SBRotationKp.getDouble(0);
      a_kI = SBRotationKi.getDouble(0);
      a_kD = SBRotationKd.getDouble(0);
      Swerve_Test_PIDCONTROLLER.setPID(a_kP, a_kI, a_kD);
    }

    
    if(SBEnablePID.getDouble(0) == 1){
      runWheelPosition(SBWheelTargettedPosition.getDouble(0));
    }

    if(SBResetPosition.getDouble(0) == 1){
      RIGHT_FRONT_DRIVE_DIRECTION_CANCoder.setPosition(0);
    }

    SBWheelPosition.setDouble(RIGHT_FRONT_DRIVE_DIRECTION_CANCoder.getPosition());
    // This method will be called once per scheduler run
    */

    SBWheelLFPosition.setDouble(LEFT_FRONT_DRIVE_WHEEL.getCurrentAngle());
    SBWheelLBPosition.setDouble(LEFT_BACK_DRIVE_WHEEL.getCurrentAngle());
    SBWheelRFPosition.setDouble(RIGHT_FRONT_DRIVE_WHEEL.getCurrentAngle());
    SBWheelRBPosition.setDouble(RIGHT_BACK_DRIVE_WHEEL.getCurrentAngle());

    SBWheelLFSpeed.setDouble(LEFT_FRONT_DRIVE_WHEEL.getCurrentSpeed());
    SBWheelLBSpeed.setDouble(LEFT_BACK_DRIVE_WHEEL.getCurrentSpeed());
    SBWheelRFSpeed.setDouble(RIGHT_FRONT_DRIVE_WHEEL.getCurrentSpeed());
    SBWheelRBSpeed.setDouble(RIGHT_BACK_DRIVE_WHEEL.getCurrentSpeed());




    if (SBEnablePIDLF.getDouble(0) ==1){
      double LFTSpeed = SBWheelLFTSpeed.getDouble(0);
      double LFTPosition =  SBWheelLFTPosition.getDouble(0);
      LEFT_FRONT_DRIVE_WHEEL.setSpeedAndDir(LFTSpeed, LFTPosition);
    }

    if (SBWheelLFTResetPosition.getDouble(0) ==1 ){
      LEFT_FRONT_DRIVE_WHEEL.resetCurrentPosition();
    }

    if (SBEnablePIDLB.getDouble(0) ==1){
      double LBTSpeed = SBWheelLBTSpeed.getDouble(0);
      double LBTPosition =  SBWheelLBTPosition.getDouble(0);
      LEFT_BACK_DRIVE_WHEEL.setSpeedAndDir(LBTSpeed, LBTPosition);
    }

    if (SBWheelLBTResetPosition.getDouble(0) ==1 ){
      LEFT_BACK_DRIVE_WHEEL.resetCurrentPosition();
    }

    if (SBEnablePIDRF.getDouble(0) ==1){
      double RFTSpeed = SBWheelRFTSpeed.getDouble(0);
      double RFTPosition =  SBWheelRFTPosition.getDouble(0);
      RIGHT_FRONT_DRIVE_WHEEL.setSpeedAndDir(RFTSpeed, RFTPosition);
    }

    if (SBWheelRFTResetPosition.getDouble(0) ==1 ){
      RIGHT_FRONT_DRIVE_WHEEL.resetCurrentPosition();
    }


    if (SBEnablePIDRB.getDouble(0) ==1){
      double RBTSpeed = SBWheelRBTSpeed.getDouble(0);
      double RBTPosition =  SBWheelRBTPosition.getDouble(0);
      RIGHT_BACK_DRIVE_WHEEL.setSpeedAndDir(RBTSpeed, RBTPosition);
    }

    if (SBWheelRBTResetPosition.getDouble(0) ==1 ){
      RIGHT_BACK_DRIVE_WHEEL.resetCurrentPosition();
    }

    if (SBResetSpeedPID.getDouble(0) ==1 ){
      Double speedP = SBSpeedPIDP.getDouble(0);
      Double speedI = SBSpeedPIDI.getDouble(0);
      Double speedD = SBSpeedPIDD.getDouble(0);

      RIGHT_BACK_DRIVE_WHEEL.setPIDSpeed(speedP,speedI,speedD);
    }

    SBGyroYaw.setDouble(DRIVE_GYRO.getYaw());
    SBGyroPitch.setDouble(DRIVE_GYRO.getPitch());
    SBGyroRoll.setDouble(DRIVE_GYRO.getRoll());

    

  }




}
 