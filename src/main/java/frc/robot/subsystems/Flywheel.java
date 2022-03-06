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
  private double flywheelControllerKP = 0.1;
  private double flywheelControllerKI = 0;
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

    // SmartDashboard.putNumber("Flywheel kP", flywheelControllerKP);
    // SmartDashboard.putNumber("Flywheel kI", flywheelControllerKI);
    // SmartDashboard.putNumber("Flywheel kD", flywheelControllerKD);
  }
  
  // public void move(double speedInEncoderUnits) {
  public void move(double output) {
    // double pidOutput = flywheelController.calculate(getRawVelocity(), speedInEncoderUnits); //somehow doesnt reach setpoint
    flywheelMotor.set(ControlMode.PercentOutput, output);

    // flywheelMotor.set(ControlMode.Velocity, speedInEncoderUnits);
  }
  
  public void moveRawPercent(double speed) {
    flywheelMotor.set(ControlMode.PercentOutput, speed);
  }
  
  public double getVelocity() //max velocity = 19776u / 100ms, which is 2896.875 rotations/min, conversion factor is 600/4096 (75/512)
  {
    double factor = 75.0 / 512.0;
    return flywheelMotor.getSelectedSensorVelocity() * factor;
  }

  public double getRawVelocity() {
    return flywheelMotor.getSelectedSensorPosition();
  }
  
  public void setFlywheelGains(double kP, double kI, double kD){
      // flywheelController.setPID(kP, kI, kD);
      flywheelMotor.config_kP(0, kP);
      flywheelMotor.config_kI(0, kI);
      flywheelMotor.config_kD(0, kD);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setFlywheelGains(SmartDashboard.getNumber("Flywheel kP", 0.0), 
      SmartDashboard.getNumber("Flywheel kI", 0.0), 
      SmartDashboard.getNumber("Flywheel kD", 0.0));
    SmartDashboard.putNumber("Flywheel Current RPM", getVelocity());
  }
}
