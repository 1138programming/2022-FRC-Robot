package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class Flywheel extends SubsystemBase {

  private TalonFX flywheelMotor;
  private PIDController flywheelController;
  private double flywheelControllerKP = 1;
  private double flywheelControllerKI = 0;
  private double flywheelControllerKD = 0;
  private boolean isMoving = false;

  public Flywheel() {
    flywheelMotor = new TalonFX(KFlywheelMotorTalon);
    flywheelController = new PIDController(flywheelControllerKP, flywheelControllerKI, flywheelControllerKD);
    // flywheelMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    flywheelMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    SmartDashboard.putNumber("Flywheel kP", flywheelControllerKP);
    SmartDashboard.putNumber("Flywheel kI", 0.0);
    SmartDashboard.putNumber("Flywheel kD", 0.0);
  }

  public void move(double speed) {
    flywheelMotor.set(ControlMode.PercentOutput, flywheelController.calculate(getVelocity(), speed));
  }
  
  public void moveRawPercent(double speed) {
    flywheelMotor.set(ControlMode.PercentOutput, speed);
  }
  
  public double getVelocity() //max velocity = 19776u / 100ms, which is 2896.875 rotations/min, conversion factor is 600/4096 (75/512)
  {
    return flywheelMotor.getSelectedSensorVelocity() * (75/512); //returns in RPM
  }
  
  public void setFlywheelGains(double kP, double kI, double kD){
    flywheelController.setPID(kP, kI, kD);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setFlywheelGains(SmartDashboard.getNumber("Flywheel kP", 0.0), 
                    SmartDashboard.getNumber("Flywheel kI", 0.0), 
                    SmartDashboard.getNumber("Flywheel kD", 0.0));
    SmartDashboard.putNumber("flywheel velocity", flywheelController.calculate(getVelocity(), 1));
    SmartDashboard.putBoolean("flywheelmoving", isMoving);
  }
}
