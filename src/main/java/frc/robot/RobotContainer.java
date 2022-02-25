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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

// Subsystems:
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Hang;
import frc.robot.subsystems.NeoBase;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Camera;

// Commands
import frc.robot.commands.Base.DriveWithJoysticks;
import frc.robot.commands.Base.DriveWithLimelight;
import frc.robot.commands.Base.MoveBase;
import frc.robot.commands.Base.ResetWheels;
import frc.robot.commands.Base.RotateToHeading;
import frc.robot.commands.Camera.LEDOff;
import frc.robot.commands.Camera.LEDOn;
import frc.robot.CommandGroups.Auton.Red1Auton;
import frc.robot.commands.Base.AimWithLimelight;
import frc.robot.commands.Base.BaseDriveLow;
import frc.robot.commands.Base.BaseDriveHigh;
import frc.robot.commands.Base.BaseStop;
import frc.robot.commands.Base.ResetGyro;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.commands.Intake.IntakeIn;
import frc.robot.commands.Intake.IntakeOut;
import frc.robot.commands.Storage.StorageStop;
import frc.robot.commands.Hang.HangStop;


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
  private final Storage storage = new Storage();
  
  // Each subsystems' commands
  private final DriveWithJoysticks driveWithJoysticks = new DriveWithJoysticks(base);
  private final DriveWithLimelight driveWithLimelight = new DriveWithLimelight(base, camera);
  private final BaseDriveLow baseDriveLow = new BaseDriveLow(base);
  private final BaseDriveHigh baseDriveHigh = new BaseDriveHigh(base);
  private final ResetGyro resetGyro = new ResetGyro(base);
  private final IntakeIn intakeIn = new IntakeIn(intake);
  private final IntakeOut intakeOut = new IntakeOut(intake);
  private final IntakeStop intakeStop = new IntakeStop(intake);
  private final HangStop hangStop = new HangStop(hang);
  private final StorageStop storageStop= new StorageStop(storage);
  private final AimWithLimelight aimWithLimelight = new AimWithLimelight(base, camera);
  private final LEDOff ledOff = new LEDOff(camera);
  private final LEDOn ledOn = new LEDOn(camera);

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
    
    xboxBtnX.whenHeld(intakeIn);
    xboxBtnY.whenHeld(intakeOut);

    logitechBtnA.whenHeld(aimWithLimelight);
    logitechBtnRT.whileHeld(driveWithLimelight);
    logitechBtnLT.whenPressed(baseDriveHigh);
    logitechBtnLT.whenReleased(baseDriveLow);
    logitechBtnY.whenPressed(resetGyro);
    logitechBtnB.whenHeld(new ResetWheels(base));
    logitechBtnX.whenHeld(ledOn);
    logitechBtnRB.whenHeld(new RotateToHeading(base, 0));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    TrajectoryConfig config;
    Trajectory trajectory1;
    Trajectory trajectory2;

    PathPlannerTrajectory test1 = PathPlanner.loadPath("Test1", 1, 1);

    SwerveControllerCommand command1;
    SwerveControllerCommand command2;
    SwerveControllerCommand test1Command;
    PPSwerveControllerCommand ppTest1;

    config = new TrajectoryConfig(1.25, 1.25);
    config.setKinematics(base.getKinematics());

    
    // trajectory1 = TrajectoryGenerator.generateTrajectory(
    //   new Pose2d(0, 0, new Rotation2d()),
    //   List.of(
    //     new Translation2d(5, 0)
    //   ),
    //   new Pose2d(5, 0, new Rotation2d()),
    //   config
    // ); 
    trajectory2 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(5, 0, new Rotation2d()),
      List.of(
        new Translation2d(-5, 0)
      ),
      new Pose2d(0, 0, new Rotation2d()),
      config
    );

    base.resetGyro();
    base.resetOdometry(test1.getInitialPose());

    ProfiledPIDController thetaController = new ProfiledPIDController(
      1, 0, 0, new TrapezoidProfile.Constraints(5, 5));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // command1 = new SwerveControllerCommand(
    //   trajectory1, 
    //   base::getPose, 
    //   base.getKinematics(), 
    //   new PIDController(1, 0, 0),
    //   new PIDController(1, 0, 0),
    //   thetaController,
    //   base::setModuleStates,
    //   base
    // );

    // command2 = new SwerveControllerCommand(
    //   trajectory2, 
    //   base::getPose, 
    //   base.getKinematics(), 
    //   new PIDController(1, 0, 0),
    //   new PIDController(1, 0, 0),
    //   thetaController,
    //   base::setModuleStates,
    //   base
    // ); 

    // test1Command = new SwerveControllerCommand(trajectory, pose, kinematics, xController, yController, thetaController, desiredRotation, outputModuleStates, requirements)
    test1Command = new SwerveControllerCommand(
      test1, 
      base::getPose, 
      base.getKinematics(), 
      new PIDController(0.8, 0, 0),
      new PIDController(0.8, 0, 0),
      thetaController,
      base::getHeading,
      base::setModuleStates,
      base
    );

    // test1Command = new SwerveControllerCommand(
    //   test1, 
    //   base::getPose, 
    //   base.getKinematics(), 
    //   new PIDController(1, 0, 0),
    //   new PIDController(1, 0, 0),
    //   thetaController,
    //   base::setModuleStates,
    //   base
    // );

    // test1Command = new SwerveControllerCommand(trajectory, pose, kinematics, xController, yController, thetaController, desiredRotation, outputModuleStates, requirements)
    // );
    
    ppTest1 = new PPSwerveControllerCommand(
      test1, 
      base::getPose, 
      base.getKinematics(), 
      new PIDController(1, 0, 0),
      new PIDController(1, 0, 0),
      thetaController,
      base::setModuleStates,
      base
    );
    

    // return testTrajectory;
    // return new ResetWheels(base);
    base.resetGyro();
    // return test1Command.andThen(() -> base.drive(0, 0, 0, false));
    // return command.andThen(new BaseStop(base));

    Trajectory red1 = PathPlanner.loadPath("Red 1 Part 1", 2, 2);
    // Trajectory red1Traj = new Trajectory(
    //   new Pose2d(),
    //   new
    // )
    Trajectory red2 = PathPlanner.loadPath("Red 1 Part 2", 2, 2);
    SwerveControllerCommand red1Command = new SwerveControllerCommand(
      red1, 
      base::getPose, 
      base.getKinematics(), 
      new PIDController(1, 0, 0),
      new PIDController(1, 0, 0),
      thetaController,
      base::getHeading,
      base::setModuleStates,
      base
    );
    SwerveControllerCommand red2Command = new SwerveControllerCommand(
      red2, 
      base::getPose, 
      base.getKinematics(), 
      new PIDController(1, 0, 0),
      new PIDController(1, 0, 0),
      thetaController,
      base::getHeading,
      base::setModuleStates,
      base
    );

    base.resetOdometry(red1.getInitialPose());
    // return red1Command.andThen(new RotateToHeading(base, 0).andThen(red2Command));
      return red2Command;
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
