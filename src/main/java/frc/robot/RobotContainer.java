// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import static frc.robot.Constants.*;

import java.lang.reflect.Array;
import java.util.List;
//wpilib
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// Subsystems:
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Hang;
import frc.robot.subsystems.NeoBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Storage;
// Commands:
import frc.robot.commands.Base.DriveWithJoysticks;
import frc.robot.commands.Base.DriveWithLimelight;
import frc.robot.commands.Flywheel.FlywheelAutoSpinUp;
import frc.robot.commands.Flywheel.FlywheelSpinAtRPM;
import frc.robot.commands.Flywheel.FlywheelSpinDefaultSpeed;
import frc.robot.commands.Flywheel.FlywheelSpinAtRPM;
import frc.robot.commands.Flywheel.FlywheelSpinWithLimelight;
import frc.robot.commands.Flywheel.FlywheelStop; 
import frc.robot.commands.Base.ResetWheels;
import frc.robot.commands.Camera.LEDOff;
import frc.robot.commands.Camera.LEDOn;
import frc.robot.commands.Base.AimWithLimelight;
import frc.robot.commands.Base.BaseDriveLow;
import frc.robot.commands.Base.BaseDriveHigh;
import frc.robot.commands.Base.BaseStop;
import frc.robot.commands.Base.DriveToPose;
import frc.robot.commands.Base.ResetGyro;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.commands.Intake.IntakeSwivelDown;
import frc.robot.commands.Intake.IntakeSwivelUp;
import frc.robot.commands.Intake.IntakeSpinBackward;
import frc.robot.commands.Intake.IntakeSpinForward;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.commands.Intake.IntakeSpinBackward;
import frc.robot.commands.Intake.HuntMode;
import frc.robot.commands.Intake.StowedMode;
import frc.robot.commands.Storage.StorageStop;
import frc.robot.commands.Storage.BottomStorageOut;
import frc.robot.commands.Storage.BottomStorageIn;
import frc.robot.commands.Storage.BottomStorageStop;
import frc.robot.commands.Storage.StorageSpinIntoFlywheel;
import frc.robot.commands.Storage.StorageCollect;
import frc.robot.commands.Storage.TopStorageOut;
import frc.robot.commands.Storage.TopStorageIn;
import frc.robot.commands.Storage.StorageOut;
import frc.robot.commands.Hang.HangStop;
import frc.robot.commands.Hang.MoveArmBackward;
import frc.robot.commands.Hang.MoveArmForward;
import frc.robot.commands.Hang.MoveClawIn;
import frc.robot.commands.Hang.MoveClawOut;
import frc.robot.commands.Hang.MoveHangDown;
import frc.robot.commands.Hang.MoveHangUp;
import frc.robot.commands.Hang.MoveLevelHangTo;
import frc.robot.commands.Hang.MoveRachetIn;
import frc.robot.commands.Hang.MoveRachetOut;
//Command Groups:
import frc.robot.CommandGroups.CollectAndIndexBalls;
import frc.robot.CommandGroups.FeedShot;
import frc.robot.CommandGroups.FlywheelLowGoalShot;
//Auton
import frc.robot.CommandGroups.Auton.DriveBackAndShoot;
import frc.robot.CommandGroups.Auton.FiveBallAuton;
import frc.robot.CommandGroups.Auton.ThreeBallAuton;
import frc.robot.CommandGroups.Auton.TwoBallAuton;
import frc.robot.CommandGroups.Auton.OptimizedFiveBallAuton;
import frc.robot.CommandGroups.Auton.OptimizedThreeBallAuton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //The robot's subsystems
  private final NeoBase base  = new NeoBase();
  private final Hang hang = new Hang();
  private final Camera camera = new Camera();
  private final Intake intake = new Intake();
  private final Flywheel flywheel = new Flywheel();
  private final Storage storage = new Storage();
  
  //Each subsystems' commands
  // Base
  private final DriveWithJoysticks driveWithJoysticks = new DriveWithJoysticks(base);
  private final DriveWithLimelight driveWithLimelight = new DriveWithLimelight(base, camera);
  private final AimWithLimelight aimWithLimelight = new AimWithLimelight(base, camera);
  private final BaseDriveLow baseDriveLow = new BaseDriveLow(base);
  private final BaseDriveHigh baseDriveHigh = new BaseDriveHigh(base);
  private final ResetGyro resetGyro = new ResetGyro(base);
  // Flywheel
  private final FlywheelSpinDefaultSpeed flywheelSpinDefaultSpeed = new FlywheelSpinDefaultSpeed(flywheel);
  private final FlywheelStop flywheelStop = new FlywheelStop(flywheel);
  private final FlywheelSpinAtRPM flywheelSpinAt1100 = new FlywheelSpinAtRPM(flywheel, 1100);
  private final FlywheelLowGoalShot flywheelLowGoalShot = new FlywheelLowGoalShot(flywheel, storage);
  private final FlywheelSpinWithLimelight flywheelSpinWithLimelight = new FlywheelSpinWithLimelight(flywheel, camera);
  private final FlywheelAutoSpinUp flywheelAutoSpinUp = new FlywheelAutoSpinUp(flywheel, camera, storage);
  // Hang
  private final HangStop hangStop = new HangStop(hang);
  private final MoveArmBackward moveArmBackward = new MoveArmBackward(hang);
  private final MoveArmForward moveArmForward = new MoveArmForward(hang);
  private final MoveClawIn moveClawIn  = new MoveClawIn(hang);
  private final MoveClawOut moveClawOut = new MoveClawOut(hang);
  private final MoveHangDown hangDown = new MoveHangDown(hang);
  private final MoveHangUp hangUp = new MoveHangUp(hang);
  private final MoveRachetIn moveRachetIn = new MoveRachetIn(hang);
  private final MoveRachetOut moveRachetOut = new MoveRachetOut(hang);

  // Intake
  private final IntakeSpinBackward intakeSpinBackward = new IntakeSpinBackward(intake);  
  private final IntakeSpinForward intakeSpinForward = new IntakeSpinForward(intake);  
  private final IntakeStop intakeStop = new IntakeStop(intake);
  private final HuntMode huntMode = new HuntMode(intake);
  private final StowedMode stowedMode = new StowedMode(intake);
  private final IntakeSwivelDown swivelDown = new IntakeSwivelDown(intake);
  private final IntakeSwivelUp swivelUp = new IntakeSwivelUp(intake);
  private final CollectAndIndexBalls collectAndIndexBalls = new CollectAndIndexBalls(intake, storage);

  // Storage
  private final StorageStop storageStop= new StorageStop(storage);
  private final BottomStorageOut bottomStorageOut = new BottomStorageOut(storage);
  private final BottomStorageIn bottomStorageIn = new BottomStorageIn(storage);
  private final BottomStorageStop bottomStorageStop = new BottomStorageStop(storage);
  private final TopStorageOut topStorageOut = new TopStorageOut(storage);
  private final TopStorageIn topStorageIn = new TopStorageIn(storage);
  private final StorageOut storageOut = new StorageOut(storage);
  private final StorageCollect storageCollect = new StorageCollect(storage);
  private final StorageSpinIntoFlywheel storageSpinIntoFlyWheel = new StorageSpinIntoFlywheel(storage);
  private final FeedShot feedShot = new FeedShot(storage);
  
  //Camera
  private final LEDOff ledOff = new LEDOff(camera);
  private final LEDOn ledOn = new LEDOn(camera);
  
  //Auton
  private final DriveBackAndShoot driveBackAndShoot = new DriveBackAndShoot(base, camera, storage, intake, flywheel);
  private final TwoBallAuton twoBallAuton = new TwoBallAuton(base, camera, storage, intake, flywheel);
  private final ThreeBallAuton threeBallAuton = new ThreeBallAuton(base, camera, storage, intake, flywheel);
  private final OptimizedThreeBallAuton optimizedThreeBallAuton = new OptimizedThreeBallAuton(base, camera, storage, intake, flywheel);
  private final FiveBallAuton fiveBallAuton = new FiveBallAuton(base, camera, storage, intake, flywheel);
  private final OptimizedFiveBallAuton optimizedFiveBallAuton = new OptimizedFiveBallAuton(base, camera, storage, intake, flywheel);

  //Controller Ports
  private static final int KLogitechPort = 0;
  private static final int KXboxPort = 1;  

  //Deadzone
  private static final double KDeadZone = 0.05;

  //Logitech Button Constants
  public static final int KLogitechButtonX = 1;
  public static final int KLogitechButtonA = 2;
  public static final int KLogitechButtonB = 3;
  public static final int KLogitechButtonY = 4;
  public static final int KLogitechLeftBumper = 5; 
  public static final int KLogitechRightBumper = 6;
  public static final int KLogitechLeftTrigger = 7;
  public static final int KLogitechRightTrigger = 8;

  private static final int KLeftYAxis = 1;
  private static final int KRightYAxis = 3;
  private static final int KLeftXAxis = 0;
  private static final int KRightXAxis = 2;

  //Xbox Button Constants
  public static final int KXboxButtonA = 1; 
  public static final int KXboxButtonB = 2;
  public static final int KXboxButtonX = 3;  
  public static final int KXboxButtonY = 4; 
  public static final int KXboxLeftBumper = 5; 
  public static final int KXboxRightBumper = 6; 
  public static final int KXboxSelectButton = 7; 
  public static final int KXboxStartButton = 8; 
  public static final int KXboxLeftTrigger = 2; 
  public static final int KXboxRightTrigger = 3; 

  //Game Controllers
  public static Joystick logitech;
  public static XboxController xbox; 
  public JoystickButton logitechBtnX, logitechBtnA, logitechBtnB, logitechBtnY, logitechBtnLB, logitechBtnRB, logitechBtnLT, logitechBtnRT; //Logitech Button
  public JoystickButton xboxBtnA, xboxBtnB, xboxBtnX, xboxBtnY, xboxBtnLB, xboxBtnRB, xboxBtnStrt, xboxBtnSelect;
  public Trigger xboxBtnRT, xboxBtnLT;

  //Misc.
  public static boolean storageBottomLimit = false;
  public static boolean storageTopLimit = false;  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //Default commands for each subsystem
    base.setDefaultCommand(driveWithJoysticks);
    hang.setDefaultCommand(hangStop);
    intake.setDefaultCommand(intakeStop);
    // flywheel.setDefaultCommand(flywheelStop);
    flywheel.setDefaultCommand(flywheelAutoSpinUp);
    intake.setDefaultCommand(intakeStop);
    storage.setDefaultCommand(storageStop);
    // camera.setDefaultCommand(ledOff);
    camera.setDefaultCommand(ledOn);

    //Game controllers
    logitech = new Joystick(KLogitechPort); //Logitech Dual Action
    xbox = new XboxController(KXboxPort);   //Xbox 360 for Windows

    // Logitch Buttons 
    logitechBtnX = new JoystickButton(logitech, KLogitechButtonX);
    logitechBtnA = new JoystickButton(logitech, KLogitechButtonA);
    logitechBtnB = new JoystickButton(logitech, KLogitechButtonB);
    logitechBtnY = new JoystickButton(logitech, KLogitechButtonY);
    logitechBtnLB = new JoystickButton(logitech, KLogitechLeftBumper);
    logitechBtnRB = new JoystickButton(logitech, KLogitechRightBumper);
    logitechBtnLT = new JoystickButton(logitech, KLogitechLeftTrigger);
    logitechBtnRT = new JoystickButton(logitech, KLogitechRightTrigger);

    // XBox Buttons
    xboxBtnA = new JoystickButton(xbox, KXboxButtonA);
  	xboxBtnB = new JoystickButton(xbox, KXboxButtonB);
		xboxBtnX = new JoystickButton(xbox, KXboxButtonX);
		xboxBtnY = new JoystickButton(xbox, KXboxButtonY);
		xboxBtnLB = new JoystickButton(xbox, KXboxLeftBumper);
    xboxBtnRB = new JoystickButton(xbox, KXboxRightBumper);
    xboxBtnSelect = new JoystickButton(xbox, KXboxSelectButton);
		xboxBtnStrt = new JoystickButton(xbox, KXboxStartButton);
    xboxBtnLT = new Trigger(() -> (joystickThreshold(xbox.getRawAxis(KXboxLeftTrigger))));
    xboxBtnRT = new Trigger(() -> (joystickThreshold(xbox.getRawAxis(KXboxRightTrigger))));
    
    // Configure the button bindings
    configureButtonBindings();

    //reset base gyro for field relative drive.
    //RobotContrainer Constructor is run in Robot init, so we put resetgyro in here
    base.resetGyro();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Drive Controls
    logitechBtnRT.whileHeld(driveWithLimelight);
    logitechBtnLT.whenPressed(baseDriveHigh);
    logitechBtnLT.whenReleased(baseDriveLow);
    logitechBtnY.whenPressed(resetGyro);
    // logitechBtnY.whenPressed(() -> base.resetOdometry(new Pose2d()));

    //Hang Controls
    logitechBtnA.whenHeld(moveClawIn);
    logitechBtnB.whenHeld(moveClawOut);  
    logitechBtnX.whenHeld(hangDown);
    logitechBtnY.whenHeld(hangUp);
    logitechBtnLB.whenHeld(moveArmForward);
    logitechBtnRB.whenHeld(moveArmBackward);
    
    // //Intake Controls
    xboxBtnY.whenHeld(swivelUp);
    xboxBtnA.whenHeld(swivelDown);
    xboxBtnLB.whenHeld(collectAndIndexBalls);
    xboxBtnLB.whenReleased(stowedMode);

    // //Storage Controls
    xboxBtnX.toggleWhenActive(flywheelStop);
    xboxBtnB.whenHeld(feedShot);
    // xboxBtnB.whenHeld(new StorageSpinIntoFlywheel(storage, 0.45));
    xboxBtnRT.whileActiveContinuous(flywheelLowGoalShot);

    // //Storage Controls
    xboxBtnLT.whileActiveContinuous(storageCollect);
    xboxBtnRB.whenHeld(storageOut);
  }

  public Command getAutonomousCommand() {
    // SmartDashboard.putString("Auton", "NoBall");
    // return null; //no auton

    SmartDashboard.putString("Auton", "OneBall");
    return twoBallAuton; //1 ball auton
    
    // SmartDashboard.putString("Auton", "TwoBall");
    // return twoBallAuton; //left / right assist auton
    
    // SmartDashboard.putString("Auton", "THreeBall");
    // return threeBallAuton; //right auton
    // return optimizedThreeBallAuton;

    // SmartDashboard.putString("Auton", "FiveBall");
    // return fiveBallAuton; //op Auton
    // return optimizedFiveBallAuton; 
  }

  public static double scaleBetween(double unscaledNum, double minAllowed, double maxAllowed, double min, double max) {
    return (maxAllowed - minAllowed) * (unscaledNum - min) / (max - min) + minAllowed;
  }
       
  public double getLogiRightYAxis() {
    final double Y = logitech.getRawAxis(KRightYAxis);
    if (Y > KDeadZone || Y < -KDeadZone)
      return -Y;
    else
      return 0;
  }

  public double getLogiLeftYAxis() {
    final double Y = logitech.getY();
    if(Y > KDeadZone || Y < -KDeadZone)
      return -Y;
    else 
      return 0; 
  }

  public double getLogiRightXAxis() {
    double X = logitech.getZ();
    if (X > KDeadZone || X < -KDeadZone) {
      return -X;
    } else {
      return 0; 
    }
  }

  public double getLogiLeftXAxis() {
    double X = logitech.getX();
    if (X > KDeadZone || X < -KDeadZone) {
      return -X;
    } else {
      return 0;
    }
  }

  public double getXboxLeftAxis() {
    final double Y = xbox.getRawAxis(KLeftYAxis);
    if(Y > KDeadZone || Y < -KDeadZone)
      return -Y;
    else 
      return 0;
  }

  public double getXboxLeftXAxis() {
    final double X = xbox.getRawAxis(KRightXAxis);
    if(X > KDeadZone || X < -KDeadZone)
      return X;
    else 
      return 0;
  }

  public double getXboxRightXAxis() {
    final double X = xbox.getRawAxis(KRightXAxis);
    if (X > KDeadZone || X < -KDeadZone)
      return -X;
    else
      return 0;
  }

  public double getXboxLeftYAxis() {
    final double Y = xbox.getRawAxis(KLeftYAxis);
    if(Y > KDeadZone || Y < -KDeadZone)
      return -Y;
    else 
      return 0;
  }

  public double getXboxRightYAxis() {
    final double Y = xbox.getRawAxis(KRightYAxis);
    if (Y > KDeadZone || Y < -KDeadZone)
      return -Y;
    else
      return 0;
  }
public boolean joystickThreshold(double triggerValue) {
    if (Math.abs(triggerValue) < .09) 
      return false;
    else 
      return true;
  }
}