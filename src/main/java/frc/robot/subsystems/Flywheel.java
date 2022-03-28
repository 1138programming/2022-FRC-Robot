package frc.robot.subsystems;

import static frc.robot.Constants.*;

import frc.robot.RobotContainer;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class Flywheel extends SubsystemBase {

  private TalonFX flywheelMotor;
  private PIDController flywheelController;
  private double flywheelControllerKP = 0.16;
  private double flywheelControllerKI = 0.001;
  private double flywheelControllerKD = 0;
  private boolean isMoving = false;

  public Flywheel() {
    flywheelMotor = new TalonFX(KFlywheelMotorTalon);
    flywheelController = new PIDController(flywheelControllerKP, flywheelControllerKI, flywheelControllerKD);
    // flywheelMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    flywheelMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    flywheelMotor.setInverted(true);

    flywheelMotor.config_kP(0, flywheelControllerKP);
    flywheelMotor.config_kI(0, flywheelControllerKI);
    flywheelMotor.config_kD(0, flywheelControllerKD);
    
    SmartDashboard.putBoolean("Flywheel Spinning", false);

    // SmartDashboard.putNumber("Flywheel kP", flywheelControllerKP);
    // SmartDashboard.putNumber("Flywheel kI", flywheelControllerKI);
    // SmartDashboard.putNumber("Flywheel kD", flywheelControllerKD);
    
    // SmartDashboard.putNumber("95", 1800);
    // SmartDashboard.putNumber("100", 1650);
    // SmartDashboard.putNumber("130", 1850);
    
  }
  
  //requires input in RPM!
  public void move(double output) {
    output *= (512.0 / 75.0); //convert to encoder unit 
    if (output != 0) {
      SmartDashboard.putBoolean("Flywheel Spinning", true);
      flywheelMotor.set(ControlMode.Velocity, output);
    }
    else {
      SmartDashboard.putBoolean("Flywheel Spinning", false);
      flywheelMotor.set(ControlMode.PercentOutput, 0);
    }
    // flywheelMotor.set(ControlMode.PercentOutput, output);
    
  }
  
  public void moveRawPercent(double speed) {
    if (speed != 0) {
      SmartDashboard.putBoolean("Flywheel Spinning", true);
    }
    else {
      SmartDashboard.putBoolean("Flywheel Spinning", false);
    }
    flywheelMotor.set(ControlMode.PercentOutput, speed);
  }
  
  public double getVelocity() //max velocity = 19776u / 100ms, which is 2896.875 rotations/min, conversion factor is 600/4096 (75/512)
  {
    double factor = 75.0 / 512.0;
    return flywheelMotor.getSelectedSensorVelocity() * factor;
  }

  public double getRawVelocity() {
    return flywheelMotor.getSelectedSensorVelocity();
  }
  
  public void setFlywheelGains(double kP, double kI, double kD){
      // flywheelController.setPID(kP, kI, kD);
      flywheelMotor.config_kP(0, kP);
      flywheelMotor.config_kI(0, kI);
      flywheelMotor.config_kD(0, kD);
  }

  public double calculateFlywheelSpeedFromDist(double distanceFromHub) {
    double flywheelOutput;
    if (distanceFromHub > 60) {
      if (distanceFromHub < 95) {
        flywheelOutput = 1850 + distanceFromHub * 4.077;
        // flywheelOutput = SmartDashboard.getNumber("95", 1850) + distanceFromHub * 4.077;
      }
      else if (distanceFromHub < 100) {
        flywheelOutput = 1750 + distanceFromHub * 4.077;
        // flywheelOutput = SmartDashboard.getNumber("100", 1750) + distanceFromHub * 4.077;
      }
      else if (distanceFromHub < 130) {
        flywheelOutput = 1850 + distanceFromHub * 4.077;
        // flywheelOutput = SmartDashboard.getNumber("130", 1850) + distanceFromHub * 4.077;
      }
      else if (distanceFromHub < 150) {
        flywheelOutput = 2200 + distanceFromHub * 4.077;
      }
      else {
        flywheelOutput = 2450 + distanceFromHub * 4.077;
      }
    }
    else {
      flywheelOutput = 1900;
    }
    return flywheelOutput;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // setFlywheelGains(SmartDashboard.getNumber("Flywheel kP", 0.0), 
    //   SmartDashboard.getNumber("Flywheel kI", 0.0), 
    //   SmartDashboard.getNumber("Flywheel kD", 0.0));

    SmartDashboard.putNumber("Flywheel Current RPM", getVelocity());
  }
}
