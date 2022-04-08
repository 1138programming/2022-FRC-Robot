package frc.robot.subsystems;

import static frc.robot.Constants.*;

//All WPILib imports
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.PIDController;

//REV Robotics Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

//Misc Imports
import com.kauailabs.navx.frc.AHRS;

public class NeoBase extends SubsystemBase {
   
  public static AHRS gyro;

  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;

  private SwerveX[] modules;

  //Base Constants
  private final double kEncoderTicksPerRotation = 4096;
  private final double kWheelDiameterMeters = Units.inchesToMeters(4);
  private final double kDriveMotorGearRatio = 1 / 6.55;
  private final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
  private final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
  private final double kAngleMotorShaftToWheelRatio = 1 / 10.285714; //1/(72/7)
  private final double kAngleEncoderRot2Deg = kAngleMotorShaftToWheelRatio * 360;
  private final double kMagEncoderPeriod = 0.04; //slower than robot code period (0.02s), which makes the mag encoder not suitable 
  private final double kticksPerRevolution = 4096;
  private final double kNeoMaxRPM = 5700; //4.62 MPS

  //Offset of each module, in degrees
  private double frontLeftOffset = -318.3;
  private double frontRightOffset = -71.5;
  private double backLeftOffset = -183.2; 
  private double backRightOffset = -237.2;

  //Max Speed of Drive Motors, default is set to Low
  private final double kPhysicalMaxDriveSpeedMPS = kDriveEncoderRPM2MeterPerSec * kNeoMaxRPM; //about 4.63 Meters Per Sec, or 15 ft/s
  private double maxDriveSpeedPercent = kBaseDriveLowSpeed;
  private double maxDriveSpeedMPS = maxDriveSpeedPercent * kPhysicalMaxDriveSpeedMPS;
  private final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
  private final double KMaxAutonSpeed = 4;

  //distance in inches of a module from the center of mass (we use a square base so only 1 number is needed)
  private double kSwerveModuleLocationFromCoM = 14.5; 

  //keeps track of the robot's position on the field using encoders, updated periodically
  private Pose2d pose;

  public NeoBase() {

    //setting up navx gyro
    gyro = new AHRS(SPI.Port.kMXP); //axis calibration and reset (OmniMount): https://pdocs.kauailabs.com/navx-mxp/installation/omnimount/

    //defining the physical position of the swerve modules
    kinematics = new SwerveDriveKinematics(
      new Translation2d(
        Units.inchesToMeters(kSwerveModuleLocationFromCoM),
        Units.inchesToMeters(kSwerveModuleLocationFromCoM)
      ),
      new Translation2d(
        Units.inchesToMeters(kSwerveModuleLocationFromCoM),
        Units.inchesToMeters(-kSwerveModuleLocationFromCoM)
      ),
      new Translation2d(
        Units.inchesToMeters(-kSwerveModuleLocationFromCoM),
        Units.inchesToMeters(kSwerveModuleLocationFromCoM)
      ),
      new Translation2d(
        Units.inchesToMeters(-kSwerveModuleLocationFromCoM),
        Units.inchesToMeters(-kSwerveModuleLocationFromCoM)
      )
    );

    //Swerve module instances init in an array (order in the array defined above)
    modules = new SwerveX[] {
      // Back Left
      new SwerveX(new CANSparkMax(backLeftDriveId, MotorType.kBrushless), new CANSparkMax(backLeftSteerId, MotorType.kBrushless), new DutyCycleEncoder(backLeftMagEncoderId), Rotation2d.fromDegrees(backLeftOffset), false), 
      // Back Right
      new SwerveX(new CANSparkMax(backRightDriveId, MotorType.kBrushless), new CANSparkMax(backRightSteerId, MotorType.kBrushless), new DutyCycleEncoder(backRightMagEncoderId), Rotation2d.fromDegrees(backRightOffset), true),
      // Front Left
      new SwerveX(new CANSparkMax(frontLeftDriveId, MotorType.kBrushless), new CANSparkMax(frontLeftSteerId, MotorType.kBrushless), new DutyCycleEncoder(frontLeftMagEncoderId), Rotation2d.fromDegrees(frontLeftOffset), false), 
      // Front Right
      new SwerveX(new CANSparkMax(frontRightDriveId, MotorType.kBrushless), new CANSparkMax(frontRightSteerId, MotorType.kBrushless), new DutyCycleEncoder(frontRightMagEncoderId), Rotation2d.fromDegrees(frontRightOffset), false) 
    };

    odometry = new SwerveDriveOdometry(kinematics, new Rotation2d());

    //Reset the gyro's heading
    gyro.reset();

  }

  /**
   * Function to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param maxSpeed Max speed for the drive motors (from 0 to 1.0).
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    xSpeed *= maxDriveSpeedMPS;
    ySpeed *= maxDriveSpeedMPS;
    rot *= kMaxAngularSpeed;
    //feeding parameter speeds into toSwerveModuleStates to get an array of SwerveModuleState objects
    SwerveModuleState[] states =
      kinematics.toSwerveModuleStates(
        fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(gyro.getAngle()))
          // ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(-gyro.getAngle()))//for testbed
          : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(states, kPhysicalMaxDriveSpeedMPS);
    //setting module states, aka moving the motors
    for (int i = 0; i < states.length; i++) {
      SwerveX module = modules[i];
      SwerveModuleState state = states[i];
      module.setDesiredState(state);
    }
}
  /**
   * Function to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param maxSpeed Max speed for the drive motors (from 0 to 1.0).
   */
  public void autonDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    xSpeed *= maxDriveSpeedMPS;
    ySpeed *= maxDriveSpeedMPS;
    rot *= kMaxAngularSpeed;
    //feeding parameter speeds into toSwerveModuleStates to get an array of SwerveModuleState objects
    SwerveModuleState[] states =
      kinematics.toSwerveModuleStates(
        fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(gyro.getAngle()))
          // ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(-gyro.getAngle()))//for testbed
          : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(states, KMaxAutonSpeed);
    //setting module states, aka moving the motors
    for (int i = 0; i < states.length; i++) {
      SwerveX module = modules[i];
      SwerveModuleState state = states[i];
      module.setDesiredState(state);
    }
}

  //recalibrates gyro offset
  public void resetGyro() {
    gyro.reset(); 
    gyro.setAngleAdjustment(0);
  }

  //maybe no work
  public void resetGyro(double offsetAngle) {
    gyro.reset();
    gyro.setAngleAdjustment(offsetAngle);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Module 0 Raw Angle", modules[0].getAngleDegRaw());
    SmartDashboard.putNumber("Module 1 Raw Angle", modules[1].getAngleDegRaw());
    SmartDashboard.putNumber("Module 2 Raw Angle", modules[2].getAngleDegRaw());
    SmartDashboard.putNumber("Module 3 Raw Angle", modules[3].getAngleDegRaw());

    // SmartDashboard.putNumber("Module 0 Drive Speed", modules[0].getDriveEncoderVel());
    // SmartDashboard.putNumber("Module 1 Drive Speed", modules[1].getDriveEncoderVel());
    // SmartDashboard.putNumber("Module 2 Drive Speed", modules[2].getDriveEncoderVel());
    // SmartDashboard.putNumber("Module 3 Drive Speed", modules[3].getDriveEncoderVel());


    //used for testing pid
    // setAllModuleGains();

    pose = odometry.update(getHeading(), getSpeeds());
    SmartDashboard.putString("Pose", pose.toString());
    SmartDashboard.putNumber("gyro", gyro.getAngle());
  }

  //move all wheels so they point forward
  public void resetWheelAngles() {
    modules[0].resetWheelAngle();
    modules[1].resetWheelAngle();
    modules[2].resetWheelAngle();
    modules[3].resetWheelAngle();
  }
  
  public void setAllModuleGains() {
    modules[0].setDriveGains();
    modules[1].setDriveGains();
    modules[2].setDriveGains();
    modules[3].setDriveGains();
  }

  //setting all relative encoders to the values of the absolute encoder on the modules
  public void resetAllRelEncoders() {
    modules[0].resetRelEncoders();
    modules[1].resetRelEncoders();
    modules[2].resetRelEncoders();
    modules[3].resetRelEncoders();
  }
  
  //check see if the wheels are reset
  public boolean getWheelsHavereset() {
    return modules[0].getWheelHasReset() && 
      modules[1].getWheelHasReset() &&
      modules[2].getWheelHasReset() &&
      modules[3].getWheelHasReset();
  }

  //setting max drive speed of all base drive motors
  public void setMaxDriveSpeedPercent(double speed) {
    maxDriveSpeedPercent = speed;
    maxDriveSpeedMPS = maxDriveSpeedPercent * kPhysicalMaxDriveSpeedMPS;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double getDriveEncoderPos(int module) {
    return modules[module].getDriveEncoderPos();
  }
  public double getAngleEncoderDeg(int module) {
    return modules[module].getAngleEncoderDeg();
  }

  public double getAngleEncoderDegWithOdometryOffset(int module, double offset) {
    return (modules[module].getAngleEncoderDeg() + offset) % 360;
  }
  
  public double getHeadingDeg() {
    return (gyro.getAngle());
    // return (-gyro.getAngle()); //for testbed
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(gyro.getAngle());
  }

  //return wheel speeds, used to set odometry.  (return negative driveEncoderVel if module is reversed (check in SwerveX[] init array), positive if not reversed)
  public SwerveModuleState[] getSpeeds() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    states[0] = new SwerveModuleState(-modules[0].getDriveEncoderVel(), modules[0].getAngleR2D());
    states[1] = new SwerveModuleState(modules[1].getDriveEncoderVel(), modules[1].getAngleR2D());
    states[2] = new SwerveModuleState(-modules[2].getDriveEncoderVel(), modules[2].getAngleR2D());
    states[3] = new SwerveModuleState(-modules[3].getDriveEncoderVel(), modules[3].getAngleR2D());

    return states;
  }

  //emmm
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    applyModuleStates(desiredStates);
  }  

  //applies wheel speeds
  public void applyModuleStates(SwerveModuleState[] desiredStates) {
    desiredStates[0].speedMetersPerSecond = -desiredStates[0].speedMetersPerSecond;
    desiredStates[0].angle = new Rotation2d(-desiredStates[0].angle.getRadians());
    desiredStates[1].speedMetersPerSecond = -desiredStates[1].speedMetersPerSecond;
    desiredStates[1].angle = new Rotation2d(-desiredStates[1].angle.getRadians());
    desiredStates[2].speedMetersPerSecond = -desiredStates[2].speedMetersPerSecond;
    desiredStates[2].angle = new Rotation2d(desiredStates[2].angle.getRadians());
    desiredStates[3].speedMetersPerSecond = -desiredStates[3].speedMetersPerSecond;
    desiredStates[3].angle = new Rotation2d(desiredStates[3].angle.getRadians());
    
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kPhysicalMaxDriveSpeedMPS);
      modules[0].setDesiredState(desiredStates[0]);
      modules[1].setDesiredState(desiredStates[1]);
      modules[2].setDesiredState(desiredStates[2]);
      modules[3].setDesiredState(desiredStates[3]);
  }
  
  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }
  public SwerveDriveOdometry getOdometry() {
    return odometry;
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    Rotation2d gyroR2D = Rotation2d.fromDegrees(gyro.getAngle());
    odometry.resetPosition(pose, gyroR2D);
  }

  class SwerveX {
    private final double KAngleP = 0.006;
    private final double KAngleI = 0;
    private final double KAngleD = 0;

    private final double KDriveP = 0.2;
    private final double KDriveI = 0.75;
    private final double KDriveD = 0.005;
    
    private CANSparkMax driveMotor;
    private CANSparkMax angleMotor;
    private DutyCycleEncoder magEncoder;
    private RelativeEncoder driveEncoder;
    private RelativeEncoder angleEncoder;
    private PIDController angleController;
    private PIDController driveController;
    private Rotation2d offset;
    private boolean isInverted;
    private double[] pulseWidthAndPeriod = new double[]{1, 1/244}; //pulse width found in mag encoder manual pdf, period is 1/frequency (also found in pdf)
    private double angleMotorOutput;
    private double driveMotorOutput;
    
    SwerveX(CANSparkMax driveMotor, CANSparkMax angleMotor, DutyCycleEncoder magEncoder, Rotation2d offset, boolean isInverted) {
      this.driveMotor = driveMotor;
      this.angleMotor = angleMotor;
      this.magEncoder = magEncoder;
      this.offset = offset;
      this.isInverted = isInverted;
      
      //PIDControllers
      angleController = new PIDController(KAngleP, KAngleI, KAngleD);
      driveController = new PIDController(KDriveP, KDriveI, KDriveD);
      
      //Telling the PIDcontroller that 180 degrees in one direction is the same as 180 degrees in the other direction.
      angleController.enableContinuousInput(-180, 180);
      
      //Sets the motor break mode to either kBreak or kCoast.
      angleMotor.setIdleMode(IdleMode.kBrake);
      driveMotor.setIdleMode(IdleMode.kBrake); 
      
      driveEncoder = driveMotor.getEncoder();
      angleEncoder = angleMotor.getEncoder();
      
      //Set relative encoders' conversion factors so they return readings in meters and degrees.
      driveEncoder.setPositionConversionFactor(kDriveEncoderRot2Meter);
      driveEncoder.setVelocityConversionFactor(kDriveEncoderRPM2MeterPerSec);
      angleEncoder.setPositionConversionFactor(kAngleEncoderRot2Deg);
    }

    public void setDriveGains() {
      driveController.setP(SmartDashboard.getNumber("kp", 0));
      driveController.setI(SmartDashboard.getNumber("ki", 0));
      driveController.setD(SmartDashboard.getNumber("kd", 0));
    }
    
    //Resets all relative encoders to match absolute encoder value, used in DriveWithJoysticks Command.
    public void resetRelEncoders() {
      driveEncoder.setPosition(0);
      angleEncoder.setPosition(getAngleDeg() - offset.getDegrees());
    }

    //Encoder get functions ()
    public double getDriveEncoderPos() {
      return driveEncoder.getPosition();
    }
    //Gets Drive Encoder Velocity (in meters per second)
    public double getDriveEncoderVel() {
      return driveEncoder.getVelocity();
    }
    //Gets Drive Encoder Velocity
    public double getDriveVelPercent() {
      return driveEncoder.getVelocity();
    }
    //Gets Angle encoder in Degrees
    public double getAngleEncoderDeg() {
      return (angleEncoder.getPosition() % 360);
    }
    //Gets Angle in R2D
    public Rotation2d getAngleR2D() {
      return Rotation2d.fromDegrees(getAngleEncoderDeg()); 
    }
    public Rotation2d getMoProAngleR2D() {
      return Rotation2d.fromDegrees(-getAngleEncoderDeg()); 
    }
    //Gets Angle in Degrees
    public double getAngleDeg() {
      double angle = -(getAbsoluteTicks() / kticksPerRevolution) * 360;
      return angle;
    }
    public double getAngleDegRaw() {
      double angle = -(getRawAbsoluteRot() * 360) % 360;
      return angle;
    }

    public double getAngleDegFromGyro() {
      return gyro.getAngle() - getAngleDeg() % 360;
    }
    //Gets Absolute Ticks
    public double getRawAbsoluteRot(){
      return magEncoder.get();
    }
    //Gets Absolute Ticks
    public double getAbsoluteTicks(){
      double magEncoderAbsValue = magEncoder.get();
      if (magEncoderAbsValue < 0)
      {
        magEncoderAbsValue = kticksPerRevolution + (magEncoder.get() % 1 ) * kticksPerRevolution;  //convert from revoltions (unit) to ticks(unit)
      }
      else {
        magEncoderAbsValue = (magEncoder.get() % 1) * kticksPerRevolution;
      }
      return magEncoderAbsValue;
    }

    public void resetWheelAngle() {
      double output = angleController.calculate(getAngleEncoderDeg(), 0);
      angleMotor.set(output);
    }

    public boolean getWheelHasReset() {
      return getAngleEncoderDeg() < .1;
    }

    //:)
    /**
     * Set the speed + rotation of the swerve module from a SwerveModuleState object
     * @param desiredState - A SwerveModuleState representing the desired new state of the module
     */
    public void setDesiredState(SwerveModuleState desiredState) {

      //If there is no controller input, sets angle and drive motor to 0.
      if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
        angleMotor.set(0);
        driveMotor.set(0);
        return;
      }

      Rotation2d currentAngleR2D = getAngleR2D();
      desiredState = SwerveModuleState.optimize(desiredState, currentAngleR2D);
      
      //Find the difference between our current rotational position and our new rotational position
      Rotation2d rotationDelta = desiredState.angle.minus(currentAngleR2D);

      //Find the new absolute position of the module based on the difference in degrees
      double deltaDeg = rotationDelta.getDegrees();

      if (Math.abs(deltaDeg) < 2) {
        angleMotorOutput = 0;
        }
      else {
        angleMotorOutput = angleController.calculate(getAngleEncoderDeg(), desiredState.angle.getDegrees());
      }  

      //comment out when testing so fingies dont get chopped off
      angleMotor.set(angleMotorOutput);

      if (isInverted) {
        // driveMotorOutput = driveController.calculate(getDriveEncoderVel(), -(desiredState.speedMetersPerSecond));
        driveMotorOutput = -desiredState.speedMetersPerSecond / kPhysicalMaxDriveSpeedMPS;
        // driveMotorOutput = MathUtil.clamp(driveMotorOutput, -1.0, 1.0);
      }
      else {
        // driveMotorOutput = driveController.calculate(getDriveEncoderVel(), (desiredState.speedMetersPerSecond));
        driveMotorOutput = desiredState.speedMetersPerSecond / kPhysicalMaxDriveSpeedMPS;
        // driveMotorOutput = MathUtil.clamp(driveMotorOutput, -1.0, 1.0);
      }

      //comment out when testing so fingies dont get chopped off
      driveMotor.set(driveMotorOutput); 
      // driveMotor.set(desiredState.speedMetersPerSecond / kPhysicalMaxDriveSpeedMPS); 
    }
  }
}