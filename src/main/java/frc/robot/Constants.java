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
    public static final int KLeftHangFalcon = 14;
    public static final int KRightHangFalcon = 15;
    public static final int KLevelHangNeo = 16;
    
    
    //Digital IO Ports
    //Base
    public static final int frontRightMagEncoderId = 4;
    public static final int frontLeftMagEncoderId = 2;
    public static final int backLeftMagEncoderId = 1;
    public static final int backRightMagEncoderId = 3;
    
    //Storage
    public static final int KStorageSensorTop = 7;
    public static final int KBallSensorBottom = 9;
    
    //Intake
    public static final int kIntakeTopLimit = 6;  
    
    //Hang
    public static final int KLeftArmLimit = 5;
    public static final int KRightArmLimit = 8;
    public static final int KHangLimit = 0; 
    
    //PWM Ports
    //Hang
    public static final int KLeftClawServo = 0;
    public static final int KRatchetServo = 2;
    public static final int KRightClawServo = 1;
    
    // Default PWM Values
    //Base
    public static final double kBaseDriveLowSpeed = 0.4;
    public static final double kBaseDriveHighSpeed = 0.7;
    public static final double kBaseDriveFullSpeed = 1.0; //about 5700 RPM
    
    //Intake
    public static final double KIntakeSpinPWM = 0.9;
    public static final double KIntakeSwivelPWM = 0.4;
    
    //FlyWheel
    public static final double KFlywheelSpeed = 0.2;
    public static final double kYOffsetDeadzone = 10;
    public static final double kXOffsetDeadzone = 10;

    //storage
    public static final double kStoragePWM = 0.5;

    //Hang 
    public static final double KArmPWM = 0.4; //Left Arm positive goes back, right arm positive goes forward
    public static final double KLevelHangSpeed = 0.3; //Negative is up.
    
    //Base Constants
    public static final double kMaxSpeed = 6.09; // 20 feet per second
    public static final double kMaxMotorOutput = 1.0;
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
    public static final double kticksPerRevolution = 4096;
    public static final double kAutonMaxDriveVelocity = 2;
    public static final double kAutonMaxAngularVelocity = (4 * Math.PI)/10;
    public static final double kAutonMaxAccel = 2;
    public static final double kAutonMaxAngularAccel = Math.PI/4;
    public static final double ks = 0.56097;
    public static final double kv = 1.8624;
    public static final double ka = 2.208;

    //Flywheel Constants
    public static final double kFlywheelMaxRPM = 3060.0; //Max Flywheel RPM From Testing

    //Intake Constants
    public static final int KIntakePos = 3350; //all the way down (encoder units)
    public static final int kStowedPos = 150; //all the way down (encoder units)

    // Limelight Constants
    public static final double KLimelightHeight = 35; // inches
    public static final double KHubHeight = 104; // inches
    public static final double KHeightDifference = KHubHeight - KLimelightHeight; // inches
    public static final double KLimelightAngle = 45;
    public static final double KLimelightRange = 29.8;
    public static final double kDesiredYOffset = 1;
    public static final double kDesiredXOffset = 1;
    public static final double kLimelightXOffsetDeadzone = 0.05;
    // public static final double kYOffsetDeadzone = 10; //not implemented yet

    //Hang Constants
    public static final double kHangRatchetDistance = 0.25;
}
