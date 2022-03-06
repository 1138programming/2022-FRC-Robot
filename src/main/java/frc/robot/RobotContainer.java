// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Array;
import java.util.List;

import com.fasterxml.jackson.databind.cfg.MapperConfigBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.links.Link;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.Pixy2.LinkType;
import io.github.pseudoresonance.pixy2api.links.SPILink;
import io.github.pseudoresonance.pixy2api.links.Link;

import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import static frc.robot.Constants.*;

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
import frc.robot.commands.Flywheel.FlywheelSpinWithLimelight;
import frc.robot.commands.Flywheel.FlywheelStop; 
import frc.robot.commands.Base.MoveBase;
import frc.robot.commands.Base.ResetWheels;
import frc.robot.commands.Base.RotateToHeading;
import frc.robot.commands.Camera.LEDOff;
import frc.robot.commands.Camera.LEDOn;
import frc.robot.CommandGroups.Auton.Red1Auton;
import frc.robot.CommandGroups.Auton.TestAuton;
import frc.robot.commands.Base.AimWithLimelight;
import frc.robot.commands.Base.BaseDriveLow;
import frc.robot.commands.Base.BaseDriveHigh;
import frc.robot.commands.Base.BaseStop;
import frc.robot.commands.Base.DriveToPose;
import frc.robot.commands.Base.ResetGyro;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.commands.Intake.IntakeSpinBackward;
import frc.robot.commands.Intake.IntakeSpinForward;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.commands.Intake.IntakeSpinBackward;
import frc.robot.commands.Intake.HuntMode;
import frc.robot.commands.Intake.IntakeMoveSwivel;
import frc.robot.commands.Intake.StowedMode;
import frc.robot.commands.Storage.StorageStop;
import frc.robot.commands.Storage.BottomStorageIn;
import frc.robot.commands.Storage.TopStorageOut;
import frc.robot.commands.Storage.TopStorageIn;
import frc.robot.commands.Hang.HangStop;
import frc.robot.commands.Hang.MoveArmBackward;
import frc.robot.commands.Hang.MoveArmForward;
import frc.robot.commands.Hang.MoveClawIn;
import frc.robot.commands.Hang.MoveClawOut;
import frc.robot.commands.Hang.MoveHangDown;
import frc.robot.commands.Hang.MoveHangUp;
import frc.robot.commands.Hang.MoveLevelHangTo;
// import frc.robot.commands.Hang.MoveRachetIn;
import io.github.pseudoresonance.pixy2api.*;


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
  private final BaseDriveLow baseDriveLow = new BaseDriveLow(base);
  private final BaseDriveHigh baseDriveHigh = new BaseDriveHigh(base);
  private final ResetGyro resetGyro = new ResetGyro(base);
  // Flywheel
  private final FlywheelSpin flywheelSpin = new FlywheelSpin(flywheel);
  private final FlywheelStop flywheelStop = new FlywheelStop(flywheel);
  private final FlywheelSpinWithLimelight flywheelSpinWithLimelight = new FlywheelSpinWithLimelight(flywheel, camera);
  // Hang
  private final HangStop hangStop = new HangStop(hang);
  private final MoveArmBackward moveArmBackward = new MoveArmBackward(hang);
  private final MoveArmForward moveArmForward = new MoveArmForward(hang);
  private final MoveClawIn moveClawIn  = new MoveClawIn(hang);
  private final MoveClawOut moveClawOut = new MoveClawOut(hang);
  private final MoveHangDown moveHangDown = new MoveHangDown(hang);
  private final MoveHangUp moveHangUp = new MoveHangUp(hang);
  // private final MoveRachetIn moveRachetIn = new MoveRachetIn(hang);

  // Intake
  private final IntakeSpinBackward intakeSpinBackward = new IntakeSpinBackward(intake);  
  private final IntakeSpinForward intakeSpinForward = new IntakeSpinForward(intake);  
  private final IntakeStop intakeStop = new IntakeStop(intake);
  private final HuntMode huntMode = new HuntMode(intake);
  private final StowedMode stowedMode = new StowedMode(intake);
  private final IntakeMoveSwivel swivelDown = new IntakeMoveSwivel(intake, true);
  private final IntakeMoveSwivel swivelUp = new IntakeMoveSwivel(intake, false);
  // Storage
  private final StorageStop storageStop= new StorageStop(storage);
  private final AimWithLimelight aimWithLimelight = new AimWithLimelight(base, camera);
  private final BottomStorageIn bottomStorageIn = new BottomStorageIn(storage);
  private final TopStorageOut topStorageOut = new TopStorageOut(storage);
  private final TopStorageIn topStorageIn = new TopStorageIn(storage);
  
  //Camera
  private final LEDOff ledOff = new LEDOff(camera);
  private final LEDOn ledOn = new LEDOn(camera);
  
  //Auton
  private final Red1Auton red1Auton = new Red1Auton(base);

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
    intake.setDefaultCommand(intakeStop);
    flywheel.setDefaultCommand(flywheelStop);
    // intake.setDefaultCommand(stowedMode);
    intake.setDefaultCommand(intakeStop);
    storage.setDefaultCommand(storageStop);
    camera.setDefaultCommand(ledOff);

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
    // logitechBtnA.whenHeld(aimWithLimelight);
    // logitechBtnRT.whileHeld(driveWithLimelight);
    // logitechBtnRT.whenPressed(new DriveToPose(base, new Pose2d()), false);
    // logitechBtnLT.whenPressed(baseDriveHigh);
    // logitechBtnLT.whenReleased(baseDriveLow);
    // logitechBtnY.whenPressed(resetGyro);
    // logitechBtnY.whileHeld(new ResetWheels(base));
    // logitechBtnY.whenPressed(() -> base.resetOdometry(new Pose2d()));

    //Hang Controls
    // logitechBtnA.whenHeld(moveClawIn);
    // logitechBtnB.whenHeld(moveClawOut);
    // logitechBtnX.whenHeld(moveHangDown);
    // logitechBtnY.whenHeld(moveHangUp);
    // logitechBtnLB.whenHeld(moveArmForward);
    // logitechBtnRB.whenHeld(moveArmBackward);
    
    //Intake Controls
    xboxBtnA.whenHeld(intakeSpinBackward);
    xboxBtnB.whenHeld(intakeSpinForward);
    xboxBtnX.whenHeld(swivelDown);
    xboxBtnY.whenHeld(swivelUp);

    //FLywheel Controls
    // xboxBtnY.toggleWhenActive(flywheelSpinWithLimelight);
    xboxBtnRB.whenHeld(topStorageIn);
    xboxBtnLB.whenHeld(bottomStorageIn);
    xboxBtnLT.whenHeld(flywheelSpin);
    // xboxBtnB.toggleWhenActive(flywheelSpin);
    // xboxBtnY.toggleWhenActive(ledOn);
    // xboxBtnX.whenHeld(huntMode);
  }

  public Command getAutonomousCommand() {
    // TrajectoryConfig config1 = new TrajectoryConfig(kAutonMaxDriveVelocity, kAutonMaxAccel);
    // TrajectoryConfig config2 = new TrajectoryConfig(kAutonMaxDriveVelocity, kAutonMaxAccel);
    // // config.setKinematics(base.getKinematics());
    // // config2.setKinematics(base.getKinematics());
    // // config.setEndVelocity(1);
    // // config2.setStartVelocity(1);
    // config1.setReversed(false);
    // config2.setReversed(true);

    // Trajectory trajectory1;
    // Trajectory trajectory2;
    PathPlannerTrajectory part1 = PathPlanner.loadPath("Blue 1 Part 1", kAutonMaxDriveVelocity, kAutonMaxAccel);
    PPSwerveControllerCommand part1Command;
    PathPlannerTrajectory part2 = PathPlanner.loadPath("Blue 1 Part 2", kAutonMaxDriveVelocity, kAutonMaxAccel);
    PPSwerveControllerCommand part2Command;

    // SwerveControllerCommand command1;
    // SwerveControllerCommand command2;
    // SwerveControllerCommand concatTrajCommand;

    PIDController xController = new PIDController(0.45, 0, 0);
    PIDController yController = new PIDController(0.4, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
      0.8, 0, 0, new TrapezoidProfile.Constraints(kAutonMaxAngularVelocity, kAutonMaxAngularAccel));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // trajectory1 = TrajectoryGenerator.generateTrajectory(
    //   new Pose2d(0, 0, new Rotation2d(0)),
    //   List.of(
    //     new Translation2d(2, 0)
    //   ),
    //   new Pose2d(2, 0, new Rotation2d(0)),
    //   config1
    // );

    
    // trajectory2 = TrajectoryGenerator.generateTrajectory(
    //   new Pose2d(0, 0, new Rotation2d(0)),
    //   List.of(
    //     new Translation2d(-2, 0)
    //   ),
    //   new Pose2d(-2, 0, new Rotation2d(0)),
    //   config2
    // );

    // var concatTraj = trajectory1.concatenate(trajectory2);
    
    // command1 = new SwerveControllerCommand(
    //   trajectory1,
    //   base::getPose,
    //   base.getKinematics(),
    //   xController,
    //   yController,
    //   thetaController,
    //   base::setModuleStates,
    //   base
    //   );
      
    // command2 = new SwerveControllerCommand(
    //   trajectory2,
    //   base::getPose,
    //   base.getKinematics(),
    //   xController,
    //   yController,
    //   thetaController,
    //   base::setModuleStates,
    //   base
    //   );
      
    // concatTrajCommand = new SwerveControllerCommand(
    //   concatTraj,
    //   base::getPose,
    //   base.getKinematics(),
    //   xController,
    //   yController,
    //   thetaController,
    //   base::setModuleStates,
    //   base
    //   );
        
    part1Command = new PPSwerveControllerCommand(
      part1, 
      base::getPose, 
      base.getKinematics(), 
      xController,
      yController,
      thetaController,
      base::setModuleStates,
      base
      );

    part2Command = new PPSwerveControllerCommand(
      part2,
      base::getPose, 
      base.getKinematics(), 
      xController,
      yController,
      thetaController,
      base::setModuleStates,
      base
    );

    return new TestAuton(base);
        
    //   // base.resetOdometry(trajectory1.getInitialPose());
      
    //   // trajectory1 = trajectory1.concatenate(trajectory2);
    //   // base.resetOdometry(red1.getInitialPose());
    //   // return command1;
    // return new SequentialCommandGroup(
    //   // new InstantCommand(() -> base.resetOdometry(trajectory1.getInitialPose())),
    //     // command1,
    //     // concatTrajCommand,
    //   // new InstantCommand(() -> base.resetOdometry(trajectory2.getInitialPose())),
    //   //   command2,
    //   new InstantCommand(() -> base.resetOdometry(part1.getInitialPose())),
    //     part1Command,
    //     part2Command,
    //   new InstantCommand(() -> base.drive(0,0,0,true)));
    // // return command1.andThen(command2);
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
}
