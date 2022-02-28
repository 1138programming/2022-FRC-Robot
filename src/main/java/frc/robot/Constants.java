// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Motors IDs
    //Base
    public static final int frontLeftDriveId = 1; 
    public static final int frontLeftSteerId = 2;
    public static final int frontRightDriveId = 3; 
    public static final int frontRightSteerId = 4; 
    public static final int backLeftDriveId = 5; 
    public static final int backLeftSteerId = 6;
    public static final int backRightDriveId = 7; 
    public static final int backRightSteerId = 8;
    
    //Storage
    public static final int KBottomStorageVictor = 9;
    public static final int KTopStorageVictor = 10;
    
    //Flywheel
    public static final int KFlywheelMotorTalon = 11;
    
    //Intake
    public static final int KSwivelIntakeTalon = 12;
    public static final int KSpinIntakeVictor = 13;

    //Hang
    public static final int KLeftHangMotor = 14;
    public static final int KRightHangMotor = 15;
    public static final int KLevelHangMotor = 16;
    
    
    //Digital IO Ports
    //Base
    public static final int frontLeftMagEncoderId = 0; 
    public static final int frontRightMagEncoderId = 1; 
    public static final int backLeftMagEncoderId = 2; 
    public static final int backRightMagEncoderId = 3; 
    
    //Storage
    public static final int KBallSensorTop = 4;
    public static final int KBallSensorBottom = 5;
    
    //Intake
    public static final int kIntakeBottomLimit = 6;  
    public static final int kIntakeTopLimit = 7;  
    
    //Hang
    public static final int KLeftArmLimit = 8;
    public static final int KRightArmLimit = 9;
    public static final int KHangLimit = 10; 
    
    //PWM Ports
    //Hang
    public static final int KLeftClawServo = 2;
    public static final int KRatchetServo = 0;
    public static final int KRightClawServo = 1;
    
    // Default PWM Values
    //Base
    public static final double kBaseDriveLowSpeed = 0.6;
    public static final double kBaseDriveMediumSpeed = 0.8;
    public static final double kBaseDriveHighSpeed = 1.0;
    
    //Intake
    public static final double KIntakeSpinPWM = 0.4;
    public static final double KIntakeSwivelPWM = 0.4;
    
    //FlyWheel
    public static final double KFlywheelSpeed = 1.0;
    public static final double kYOffsetDeadzone = 10;
    public static final double kXOffsetDeadzone = 10;

    //storage
    public static final double kStoragePWM = 0.5;

    //Hang 
    public static final double KArmPWM = 0.5; //Left Arm positive goes back, right arm positive goes forward
    public static final double KLevelHangSpeed = 0.45; //Negative is up.
    
    //Base Constants
    public static final double kMaxSpeed = 6.09; // 20 feet per second
    public static final double kMaxMotorOutput = 1.0;
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
    public static final double kticksPerRevolution = 4096;
    public static double fieldCalibration = 0;
    public static double frontLeftOffset = 0;
    public static double frontRightOffset = 0;
    public static double backLeftOffset = 0;
    public static double backRightOffset = 0;

    //Intake Constants
    public static final int KIntakeAngle = 45;

    
    // Limelight Constants
    public static final double KLimelightHeight = 18.75; // inches
    public static final double KHubHeight = 104; // inches
    public static final double KHeightDifference = KHubHeight - KLimelightHeight; // inches
    public static final double KLimelightAngle = 45;
    public static final double KLimelightRange = 29.8;
    public static final double kDesiredYOffset = 1;
    public static final double kDesiredXOffset = 1;
    public static final double kLimelightXOffsetDeadzone = 0.05;
    // public static final double kYOffsetDeadzone = 10; //not implemented yet
}
