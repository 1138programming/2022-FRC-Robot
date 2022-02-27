package frc.robot.subsystems;

import static frc.robot.Constants.*;

//All WPILib imports
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;

//REV Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//NavX Imports
import com.kauailabs.navx.frc.AHRS;

public class NeoBase extends SubsystemBase {
   
  public static AHRS gyro;

  private SwerveDriveKinematics kinematics;

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
  private final double kMaxSpeed = 6.09; // 20 feet per second
  private final double kMaxMotorOutput = 0.4;
  private final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
  private final double kticksPerRevolution = 4096;

  //offset of each module, in degrees
  private double frontLeftOffset = -1.0;
  private double frontRightOffset = -152.5;
  private double backLeftOffset = -103.5; 
  private double backRightOffset = -14.5;

  //Max Speed of Drive Motors, default is 0.8
  private static double maxDriveSpeed = 0.8;

  //distance in inches of a module from the center of mass (we use a square base so only 1 number is needed)
  private double kSwerveModuleLocationFromCoM = 14.5; 
  
  public NeoBase() {

    gyro = new AHRS(SPI.Port.kMXP);

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

    //swerve module instances init in an array (order in the array defined above)
    modules = new SwerveX[] {
      // Back Left
      new SwerveX(new CANSparkMax(backLeftDriveId, MotorType.kBrushless), new CANSparkMax(backLeftSteerId, MotorType.kBrushless), new DutyCycleEncoder(backLeftMagEncoderId), Rotation2d.fromDegrees(backLeftOffset), true), 
      // Back Right
      new SwerveX(new CANSparkMax(backRightDriveId, MotorType.kBrushless), new CANSparkMax(backRightSteerId, MotorType.kBrushless), new DutyCycleEncoder(backRightMagEncoderId), Rotation2d.fromDegrees(backRightOffset), true),
      // Front Left
      new SwerveX(new CANSparkMax(frontLeftDriveId, MotorType.kBrushless), new CANSparkMax(frontLeftSteerId, MotorType.kBrushless), new DutyCycleEncoder(frontLeftMagEncoderId), Rotation2d.fromDegrees(frontLeftOffset), false), 
      // Front Right
      new SwerveX(new CANSparkMax(frontRightDriveId, MotorType.kBrushless), new CANSparkMax(frontRightSteerId, MotorType.kBrushless), new DutyCycleEncoder(frontRightMagEncoderId), Rotation2d.fromDegrees(frontRightOffset), false) 
    };

  //Resets the Gyro sensor
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
  
  //feeding parameter speeds into toSwerveModuleStates to get an array of SwerveModuleState objects
  SwerveModuleState[] states =
    kinematics.toSwerveModuleStates(
      fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(-gyro.getAngle()))
        : new ChassisSpeeds(xSpeed, ySpeed, rot));
  SwerveDriveKinematics.desaturateWheelSpeeds(states, maxDriveSpeed);
  
  //setting module states, aka moving the motors
  for (int i = 0; i < states.length; i++) {
    SwerveX module = modules[i];
    SwerveModuleState state = states[i];
    module.setDesiredState(state);
  }
}

  public void resetGyro() {
    gyro.reset(); //recalibrates gyro offset
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Front Left Absolute Angle", modules[0].getAngleDeg());
    SmartDashboard.putNumber("Right Front abs Angle", modules[1].getAngleDeg());
    SmartDashboard.putNumber("Left Back abs Angle", modules[2].getAngleDeg());
    SmartDashboard.putNumber("Right Back abs Angle", modules[3].getAngleDeg());
    // This method will be called once per scheduler run
  }

  //setting all relative encoders to the values of the absolute encoder on the modules
  public void resetAllRelEncoders() {
    modules[0].resetRelEncoders();
    modules[1].resetRelEncoders();
    modules[2].resetRelEncoders();
    modules[3].resetRelEncoders();
  }

  //setting max drive speed of all base drive motors
  public void setMaxDriveSpeed(double speed) {
    maxDriveSpeed = speed;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  class SwerveX {
    
    private CANSparkMax driveMotor;
    private CANSparkMax angleMotor;
    private DutyCycleEncoder magEncoder;
    private RelativeEncoder driveEncoder;
    private RelativeEncoder angleEncoder;
    private PIDController angleController;
    private Rotation2d offset;
    private boolean isInverted;
    private double[] pulseWidthAndPeriod = new double[]{1, 1/244}; //pulse width found in mag encoder manual pdf, period is 1/frequency (also found in pdf)
    private double angleMotorOutput;
    private final double kAngleP = 0.006;
    private final double kAngleI = 0;
    private final double kAngleD = 0;
    
    SwerveX(CANSparkMax driveMotor, CANSparkMax angleMotor, DutyCycleEncoder magEncoder, Rotation2d offset, boolean isInverted) {
      this.driveMotor = driveMotor;
      this.angleMotor = angleMotor;
      this.magEncoder = magEncoder;
      this.offset = offset;
      this.isInverted = isInverted;
      
      //PIDControllers
      angleController = new PIDController(kAngleP, kAngleI, kAngleD);
      
      //Telling the PIDcontroller that 360 degrees in one direction is the same as 360 degrees in the other direction.
      angleController.enableContinuousInput(-180, 180);
      
      //Sets the motor break mode to either kBreak or kCoast.
      angleMotor.setIdleMode(IdleMode.kBrake);
      driveMotor.setIdleMode(IdleMode.kBrake); 
      
      driveEncoder = driveMotor.getEncoder();
      angleEncoder = angleMotor.getEncoder();
      
      //Set relative encoders' conversion factors so they return readings in meters and degrees.
      driveEncoder.setPositionConversionFactor(kDriveEncoderRot2Meter);
      angleEncoder.setPositionConversionFactor(kAngleEncoderRot2Deg);
    }
    
    //Resets all relative encoders to match absolute encoder value, used in DriveWithJoysticks Command.
    public void resetRelEncoders() {
      driveEncoder.setPosition(0);
      angleEncoder.setPosition(getAngleDeg() - offset.getDegrees());
    }

    //Encoder get functions
    public double getDriveEncoderPos() {
      return driveEncoder.getPosition();
    }
    //Gets Drive Encoder Velocity
    public double getDriveEncoderVel() {
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
    //Gets Angle in Degrees
    public double getAngleDeg() {
      double angle = -(getAbsoluteTicks() / kticksPerRevolution) * 360;
      return angle;
    }
    //Gets Absolute Ticks
    public double getRawAbsoluteTicks(){
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


    //:)
    /**
     * Set the speed + rotation of the swerve module from a SwerveModuleState object
     * @param desiredState - A SwerveModuleState representing the desired new state of the module
     */
    public void setDesiredState(SwerveModuleState desiredState) {

    Rotation2d currentAngleR2D = getAngleR2D();

    //If there is no controller input, sets angle and drive motor to 0.
    if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
      angleMotor.set(0);
      driveMotor.set(0);
      return;
    }
    
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

    double driveOutput = desiredState.speedMetersPerSecond;
    if (isInverted) {
      driveOutput = -driveOutput;
    }
    //comment out when testing so fingies dont get chopped off
    driveMotor.set(driveOutput); 
    }
  }
}
