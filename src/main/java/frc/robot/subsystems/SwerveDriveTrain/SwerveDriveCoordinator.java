// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveDriveTrain;

import org.opencv.core.Mat;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.Tracer;

import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class SwerveDriveCoordinator
{
    AHRS gyro;
    SwerveDriveWheel leftFrontWheel;
    SwerveDriveWheel leftBackWheel;
    SwerveDriveWheel rightFrontWheel;
    SwerveDriveWheel rightBackWheel;

    public SwerveDriveCoordinator(AHRS gyro, SwerveDriveWheel leftFrontWheel, SwerveDriveWheel leftBackWheel, SwerveDriveWheel rightFrontWheel, SwerveDriveWheel rightBackWheel)
    {
        this.gyro = gyro;
        this.leftFrontWheel = leftFrontWheel;
        this.leftBackWheel = leftBackWheel;
        this.rightFrontWheel = rightFrontWheel;
        this.rightBackWheel = rightBackWheel;

        this.leftFrontWheel.setReverted(false);
        this.rightBackWheel.setReverted(false);
        this.leftFrontWheel.setReverted(false);
        this.rightBackWheel.setReverted(false);

        leftBackWheel.resetCurrentPosition();
        rightBackWheel.resetCurrentPosition();
        leftFrontWheel.resetCurrentPosition();
        rightFrontWheel.resetCurrentPosition();

    }

    // public void translate(double direction, double power)
    // {

        

    //     leftFrontWheel.setDirection(direction);
    //     leftBackWheel.setDirection(direction);
    //     rightFrontWheel.setDirection(direction);
    //     rightBackWheel.setDirection(direction);

    //     leftFrontWheel.setSpeed(power);
    //     leftBackWheel.setSpeed(power);
    //     rightFrontWheel.setSpeed(power);
    //     rightBackWheel.setSpeed(power);

    //     leftBackWheel.setReverted(false);
    //     leftFrontWheel.setReverted(false);
    //     rightBackWheel.setReverted(false);
    //     rightFrontWheel.setReverted(false);

    //     leftBackWheel.setDirectionReverted(false);
    //     leftFrontWheel.setDirectionReverted(false);
    //     rightFrontWheel.setDirectionReverted(false);
    //     rightBackWheel.setDirectionReverted(false);

    // }
 

    //     public void inplaceTurn(double power)
    // {
    //     leftFrontWheel.setDirection(-45);
    //     leftBackWheel.setDirection(45);
    //     rightFrontWheel.setDirection(-135);
    //     rightBackWheel.setDirection(135);

    //     leftFrontWheel.setSpeed(power);
    //     leftBackWheel.setSpeed(power);
    //     rightFrontWheel.setSpeed(power);
    //     rightBackWheel.setSpeed(power);
    // }

    private static double closestAngle(double a, double b)
    {
            // get direction
            double dir = (b%360.0) - (a%360.0);

            // convert from -360 to 360 to -180 to 180
            if (Math.abs(dir) > 180.0)
            {
                    dir = -(Math.signum(dir) * 360.0) + dir;
            }
            return dir;
    }

    
    private void translation(double x, double y){
        double direction = Math.atan2(x, y)*(180/Math.PI) + gyro.getYaw();
        double speed = MathUtil.applyDeadband(Math.hypot(x, y), 0.05)* 0.2;

        leftFrontWheel.setSpeedAndDir(speed, direction);
        leftBackWheel.setSpeedAndDir(speed, direction);
        rightFrontWheel.setSpeedAndDir(speed, direction);
        rightBackWheel.setSpeedAndDir(speed, direction);

    }

    private void translationAndTurn(double _x, double _y, double rotation, double speedPercentage){
        double direction = Math.atan2(_x, _y) * (180/Math.PI) + gyro.getYaw();
        
        if (direction >180){

            direction = (direction-360);
        }
        else if (direction<=-180){
            direction = 360+ direction;
        }

        double speed = MathUtil.applyDeadband(Math.hypot(_x, _y), 0.05) * speedPercentage;

        double x = Math.sin(direction * Math.PI / 180) * speed;
        double y = Math.cos(direction * Math.PI / 180) * speed;
        //135, 45, -45, -135
        double[] angles = {-135, -45, 45, 135};
        SwerveDriveWheel[] modules = {rightFrontWheel, leftFrontWheel, leftBackWheel, rightBackWheel};

        for(int i = 0; i < 4; i++){

            double inplaceAngle = angles[i];
            double inplacePower = rotation*(speedPercentage);

            double fx = x+  Math.sin(inplaceAngle *(Math.PI/180) ) * inplacePower;
            double fy = y+ Math.cos(inplaceAngle*(Math.PI/180) ) * inplacePower;

            double fpower = Math.hypot(fx, fy);
            double fdirection = Math.atan2(fx, fy)*(180/Math.PI);
            // System.out.println("Angle Before: " + Math.round(direction) + "Angle After: "+ Math.round(fdirection));

            modules[i].setSpeedAndDir(fpower, fdirection);
            //modules[i].setSpeedAndDirBuiltIn(fpower, fdirection);

        }

    }

    private void rotate(double turnPower, double speedPercentage){
        double[] angles = {-135, -45, 45, 135};
        SwerveDriveWheel[] modules = {rightFrontWheel, leftFrontWheel, leftBackWheel, rightBackWheel};

        for(int i = 0; i < 4; i++){
            double direction = angles[i];
            double power =  turnPower *speedPercentage;

            modules[i].setSpeedAndDir(power, direction);
           //modules[i].setSpeedAndDirBuiltIn(power, direction);

        }
    }

    public void setSwerveDrive(double x, double y, double turnPower, double speedPercentage)
    {
        // translation(x, y);        
        //rotate(turnPower);
        
         if ((x == 0.0) && (y == 0) && (turnPower != 0.0))
         {
             rotate(turnPower,speedPercentage);
         }
         else
         {
            translationAndTurn(x, y,turnPower,speedPercentage); 
         }
         //rightBackWheel.setSpeedAndDir(0, 0);
    }

    public double getHeading(){
        return Math.IEEEremainder(gyro.getAngle(),360);
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }
}