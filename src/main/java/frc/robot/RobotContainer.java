// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;

// Subsystems:
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Hang;
import frc.robot.subsystems.NeoBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Storage;

// Commands
import frc.robot.commands.Base.DriveWithJoysticks;
import frc.robot.commands.Base.DriveWithLimelight;
import frc.robot.commands.Flywheel.FlywheelSpin;
import frc.robot.commands.Flywheel.FlywheelStop; 
import frc.robot.commands.Base.AimWithLimelight;
import frc.robot.commands.Base.BaseDriveLow;
import frc.robot.commands.Base.BaseDriveHigh;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.commands.Intake.IntakeIn;
import frc.robot.commands.Intake.IntakeOut;
import frc.robot.commands.Storage.StorageStop;
import frc.robot.commands.Storage.BottomStorageIn;
import frc.robot.commands.Storage.TopStorageOut;
import frc.robot.commands.Storage.TopStorageIn;
import frc.robot.commands.Hang.HangMove;
import frc.robot.commands.Hang.HangServoMove;
import frc.robot.commands.Hang.HangStop;
import frc.robot.commands.Hang.HangServoStop;
import frc.robot.commands.Hang.MoveArms;
import frc.robot.commands.Hang.MoveArmsToLimit;
import frc.robot.commands.Hang.MoveClaw;
import frc.robot.commands.Hang.MoveLift;
import frc.robot.commands.Hang.MoveLiftToBottomLimit;
import frc.robot.commands.Hang.MoveLiftToTopLimit;





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
  
  // Each subsystems' commands
  private final DriveWithJoysticks driveWithJoysticks = new DriveWithJoysticks(base);
  private final DriveWithLimelight driveWithLimelight = new DriveWithLimelight(base, camera);
  private final BaseDriveLow baseDriveLow = new BaseDriveLow(base);
  private final BaseDriveHigh baseDriveHigh = new BaseDriveHigh(base);
  private final IntakeIn intakeIn = new IntakeIn(intake);
  private final IntakeOut intakeOut = new IntakeOut(intake);
  private final IntakeStop intakeStop = new IntakeStop(intake);
  private final HangMove hangMove = new HangMove(hang);
  private final HangStop hangStop = new HangStop(hang);
  private final HangServoMove hangServoMove = new HangServoMove(hang);
  private final HangServoStop hangServoStop = new HangServoStop(hang);
  private final MoveArms moveArms = new MoveArms(hang);
  private final MoveArmsToLimit moveArmsToLimit = new MoveArmsToLimit(hang);
  private final MoveClaw moveClaw = new MoveClaw(hang);
  private final MoveLiftToBottomLimit moveLiftToBottomLimit = new MoveLiftToBottomLimit(hang);
  private final MoveLiftToTopLimit moveLiftToTopLimit = new MoveLiftToTopLimit(hang);
  private final StorageStop storageStop= new StorageStop(storage);
  private final AimWithLimelight aimWithLimelight = new AimWithLimelight(base, camera);
  private final FlywheelSpin flywheelSpin = new FlywheelSpin(flywheel);
  private final FlywheelStop flywheelStop = new FlywheelStop(flywheel);
  private final BottomStorageIn bottomStorageIn = new BottomStorageIn(storage);
  private final TopStorageOut topStorageOut = new TopStorageOut(storage);
  private final TopStorageIn topStorageIn = new TopStorageIn(storage);

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
  public static final int KXboxLeftTrigger = 9; 
  public static final int KXboxRightTrigger = 10; 

  //Game Controllers
  public static Joystick logitech;
  public static XboxController xbox; 
  public JoystickButton logitechBtnX, logitechBtnA, logitechBtnB, logitechBtnY, logitechBtnLB, logitechBtnRB, logitechBtnLT, logitechBtnRT; //Logitech Button
  public JoystickButton xboxBtnA, xboxBtnB, xboxBtnX, xboxBtnY, xboxBtnLB, xboxBtnRB, xboxBtnStrt, xboxBtnSelect, xboxBtnLT, xboxBtnRT;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //Default commands for each subsystem
    base.setDefaultCommand(driveWithJoysticks);
    hang.setDefaultCommand(hangStop);
    hang.setDefaultCommand(hangServoStop);
    intake.setDefaultCommand(intakeStop);
    flywheel.setDefaultCommand(flywheelStop);
    storage.setDefaultCommand(storageStop);

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
		xboxBtnLT = new JoystickButton(xbox, KXboxLeftTrigger);
    xboxBtnRT = new JoystickButton(xbox, KXboxRightTrigger);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    xboxBtnX.whenHeld(intakeIn);
    xboxBtnY.whenHeld(intakeOut);
    xboxBtnB.whenHeld(flywheelSpin);

    logitechBtnA.whenHeld(aimWithLimelight);
    logitechBtnRT.whileHeld(driveWithLimelight);
    logitechBtnLT.whenPressed(baseDriveLow);
    logitechBtnLT.whenReleased(baseDriveHigh);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
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
}
