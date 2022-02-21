// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

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
    public static final int frontRightDriveId = 1; 
    public static final int frontRightSteerId = 2; 
    public static final int frontLeftDriveId = 3; 
    public static final int frontLeftSteerId = 4;
    public static final int backLeftDriveId = 5; 
    public static final int backLeftSteerId = 6;
    public static final int backRightDriveId = 7; 
    public static final int backRightSteerId = 8;
    public static final int frontRightMagEncoderId = 0;
    public static final int frontLeftMagEncoderId = 1;
    public static final int backLeftMagEncoderId = 2;
    public static final int backRightMagEncoderId = 3;

    public static final double ks = 0.56097;
    public static final double kv = 1.8624;
    public static final double ka = 2.208;
    
    //Storage
    public static final int KStorageSpark = 9;

    //Shooter
    public static final int KLeftShooterMotor = 11;
    public static final int KRightShooterMotor = 12;

    //Intake
    public static final int KIntakeMotor = 10;

    //Hang
    public static final int KLeftHangMotor = 15;
    public static final int KRightHangMotor = 17;
    public static final int KMiddleHangMotor = 19;

    public static final int KLeftLinearServo = 1;
    public static final int KMiddleLinearServo = 2;
    public static final int KRightLinearServo = 3;

// Default PWM Values

    //Base
    public static final double kBaseDriveLowSpeed = 0.4;
    public static final double kBaseDriveMediumSpeed = 0.8;
    public static final double kBaseDriveHighSpeed = 1.0; //about 5700 RPM
    

    //Intake
    public static final double KIntakePWM = 1.0;

    // Limelight 
    public static final double KLimelightRange = 29.8;
    public static final double kDesiredYOffset = 1;
    public static final double kDesiredXOffset = 1;
    public static final double kXOffsetDeadzone = 0.05;
    public static final double KLimelightRange = 29.8;
    

    public static final double KLimelightHeight = 18.75; // inches
    public static final double KHubHeight = 104; // inches
    public static final double KHeightDifference = KHubHeight - KLimelightHeight; // inches
    public static final double KLimelightAngle = 45;

    // public static final double kYOffsetDeadzone = 10; //not implemented yet

}
