// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Robotmap {
  public static boolean troubleshooting = false;
  public static class SwerveDrive {
    public static int LEFT_FRONT_DRIVE_SPEED_MOTOR_PIN = 25; //28
    public static int LEFT_BACK_DRIVE_SPEED_MOTOR_PIN = 27; //26
    public static int RIGHT_FRONT_DRIVE_SPEED_MOTOR_PIN = 28; //25
    public static int RIGHT_BACK_DRIVE_SPEED_MOTOR_PIN = 26; //26 
   
    public static int LEFT_FRONT_DRIVE_DIRECTION_MOTOR_PIN = 8; //12
    public static int LEFT_BACK_DRIVE_DIRECTION_MOTOR_PIN = 10; //7
    public static int RIGHT_FRONT_DRIVE_DIRECTION_MOTOR_PIN = 12; //8
    public static int RIGHT_BACK_DRIVE_DIRECTION_MOTOR_PIN = 6;//7; //10

    public static int LEFT_FRONT_DRIVE_DIRECTION_ENCODER_PIN = 17; //16
    public static int LEFT_BACK_DRIVE_DIRECTION_ENCODER_PIN = 15; //18
    public static int RIGHT_FRONT_DRIVE_DIRECTION_ENCODER_PIN = 16; //17
    public static int RIGHT_BACK_DRIVE_DIRECTION_ENCODER_PIN = 18; //15

    public static double DRIVE_DIRECTION_P =  0.02;
    public static double DRIVE_DIRECTION_I = 0.001;
    public static double DRIVE_DIRECTION_D = 0;

    
    public static double LEFT_FRONT_DRIVE_DIRECTION_ENCODER_Offset = 58.623; //16
    public static double LEFT_BACK_DRIVE_DIRECTION_ENCODER_Offset = -151.61; //18
    public static double RIGHT_FRONT_DRIVE_DIRECTION_ENCODER_Offset = -87.36; //17
    public static double RIGHT_BACK_DRIVE_DIRECTION_ENCODER_Offset = -49.921; //15

      // Locations for the swerve drive modules relative to the robot center.

    public static Translation2d m_frontLeftLocationTranslation2d = new Translation2d(0.381, 0.381);
    public static Translation2d  m_frontRightLocatioTranslation2d = new Translation2d(0.381, -0.381);
    public static Translation2d  m_backLeftLocationTranslation2d= new Translation2d(-0.381, 0.381);
    public static Translation2d  m_backRightLocationTranslation2d = new Translation2d(-0.381, -0.381);

    public static SwerveDriveKinematics  swervedriveKinematic =new SwerveDriveKinematics(
        m_frontLeftLocationTranslation2d, m_frontRightLocatioTranslation2d, m_backLeftLocationTranslation2d, m_backRightLocationTranslation2d);


   
  }

  public static class Wrists{
    public static  int wristMotorID = 14;
    public static boolean wristTroubleShooting = false;
    public static double PIDP = 0.035;
    public static double PIDI =0.0000125;
    public static double PIDD = 0.005;

    
  }

  public static class Intake{
    public static  int intakeMotorID = 15;
    public static boolean intakeTroubleShooting = false;


    
  }

  public static class Pivot{
    public static int pivotMotorID = 30;
    public static boolean pivotTroubleShooting = false;
    public static double PIDP = 0.035;
    public static double PIDI = 0.0000125;
    public static double PIDD = 0.005;

    
  }
  public static class Extension{
    public static  int extensionMotorID = 13;
    public static boolean extensionTroubleShooting = false;
    public static double PIDP =  0.02;
    public static double PIDI = 0.00125;
    public static double PIDD = 0;

    
  }
}
