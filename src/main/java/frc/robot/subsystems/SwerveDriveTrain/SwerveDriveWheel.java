// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveDriveTrain;

import org.opencv.core.Mat;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenixpro.StatusSignalValue;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

/** Add your docs here. */
public class SwerveDriveWheel {

    public PIDController directionController;
    public PIDController directionControllerBuiltIn;

    public CANCoder directionCANcoder;
    public CANSparkMax directionMotor;
    public WPI_TalonFX speedMotor;
    public PIDController speedController;
    public double cancoderoffset;
   // public PIDOutput directionMotor;
   // public PIDSource directionSensor;

   public SwerveDriveWheel(double P, double I, double D,CANCoder dirCANCoder, CANSparkMax dirMotor,WPI_TalonFX speedMotor, double offset)
   {
        directionMotor = dirMotor;
        directionMotor.setInverted(false);
        this.directionCANcoder =  dirCANCoder;
        this.speedMotor = speedMotor;
        this.speedMotor.configClosedloopRamp(0.3);
       this.directionController = new PIDController(P, I, D);
       this.speedController = new PIDController(1, 0, 0);
       this.directionController.enableContinuousInput(-180, 180);
       this.cancoderoffset = offset;
       this.directionCANcoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        directionMotor.getEncoder().setPosition(0);



   }
   public Double getCurrentAngle(){
        double angles =  directionCANcoder.getAbsolutePosition()-cancoderoffset;

        if (angles >180){
            angles = (angles-360);
        }
        else if (angles<=-180){
            angles = 360+ angles;
        }
        

        return angles;
   }
    public double getCurrentAngleRad(){
        return directionCANcoder.getPosition()/180*Math.PI;
   }

   public double getBuiltInEncoderPosition(){
    double angle = directionMotor.getEncoder().getPosition(); 
    if (angle > 10.6){
        angle -= 21.2;
        directionMotor.getEncoder().setPosition(angle);
    }

    else if (angle < -10.6){
        angle += 21.2;
        directionMotor.getEncoder().setPosition(angle);
    }

    return angle;

    }
    SwerveModuleState getState(){
    return new SwerveModuleState(getCurrentSpeedPosition(),new Rotation2d(getCurrentAngleRad()));
   }

   public SwerveModulePosition gModulePosition(){
    return new SwerveModulePosition(getCurrentSpeedPosition(),new Rotation2d(getCurrentAngleRad()));
   }

   
   public Double getCurrentSpeed(){
    return speedMotor.getSelectedSensorVelocity();

   }

   public double getCurrentSpeedPosition(){
    return speedMotor.getSelectedSensorPosition();
   }

   public void resetCurrentPosition(){
    directionCANcoder.setPosition(0);
   }


//    public void setDirection(double setpoint)
//    {
//         double currentAngle = getCurrentAngle();
//         // double calculate = MathUtil.clamp(directionController.calculate(directionCANcoder.getPosition(), currentAngle + closestAngle(currentAngle, setpoint)), -0.75, 0.75);
//         // directionMotor.set(-calculate);

//         double calculate = MathUtil.clamp(directionController.calculate(directionCANcoder.getAbsolutePosition(), currentAngle + closestAngle(currentAngle, setpoint)), -0.75, 0.75);
//         directionMotor.set(-calculate);

       
//        // double currentAngle = getCurrentAngle();
//        //directionController.setSetpoint(currentAngle + closestAngle(currentAngle, setpoint));
//     }

    public void setSpeedAndDir(double _speed, double _dir){

        double currentAngle = getCurrentAngle();
        double speed = _speed;

        
        

        if(Math.abs(currentAngle -_dir)>90){
            
            if (_dir >0){
                _dir -=180;
            }
            else{
                _dir+=180;
            }

            speed *=-1;
        }
        

        double calculate = MathUtil.clamp(directionController.calculate(getCurrentAngle(), _dir), -0.75, 0.75);
        directionMotor.set(-calculate);

        speedMotor.set(speed);
    }

    public double angleToBuiltInPosition(double in){
        double position = in*(10.6/180);
        return position;
   
       }
    public double getBuiltInEncoderPositionToAngle(double in){
        double angle = directionMotor.getEncoder().getPosition(); 
        if (angle > 10.6){
                angle -= 21.2;
                directionMotor.getEncoder().setPosition(angle);
            }

            else if (angle  < -10.6){
                angle += 21.2;
                directionMotor.getEncoder().setPosition(angle);
            }
        return angle*(180/10.6);

    }

    public void setSpeedAndDirBuiltIn(double _speed, double _dir){

        // double currentAngle = getBuiltInEncoderPosition();
        // double targetangle = angleToBuiltInPosition(_dir);
        // double targetspeed = _speed;
        

        // if(Math.abs(currentAngle -targetangle)>5.3){
            
        //     if (targetangle >=0){
        //         targetangle -=10.6;
        //     }
        //     else{
        //         targetangle+=10.6;
        //     }

        //     targetspeed *=-1;
        // }



        // double calculate = MathUtil.clamp(directionController.calculate(targetangle,currentAngle), -1, 1);
        // directionMotor.set(-calculate);

        // speedMotor.set(targetspeed);

        double currentAngle = getBuiltInEncoderPositionToAngle(_dir);
        double targetangle = _dir;
        double targetspeed = _speed;
        

        if(Math.abs(currentAngle -_dir)>90){
            
            if (_dir >=0){
                _dir -=180;
            }
            else{
                _dir+=180;
            }

            _speed *=-1;
        }

        double calculate = MathUtil.clamp(directionController.calculate(targetangle, currentAngle), -0.75, 0.75);
        directionMotor.set(-calculate);

        speedMotor.set(targetspeed);
    }

    public void setReverted(boolean revert){
        speedMotor.setInverted(revert);
    }

    public void setDirectionReverted(boolean revert){
        directionMotor.setInverted(revert);
    }

  
    

    public void setPIDSpeed(double p, double i, double d){
        speedController.setPID(p, i, d);
    }


    public void setSpeed(double speed)
    {
        //double speedcalculate = MathUtil.clamp(speedController.calculate(getCurrentSpeed(), (speed)*12500 ) , -0.75, 0.75);
        speedMotor.set(speed);
    }



}
