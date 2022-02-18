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
  private double flywheelControllerKI = 1;
  private double flywheelControllerKD = 1;


  public Flywheel() {
    flywheelMotor = new TalonFX(KFlywheelMotor);
    flywheelController = new PIDController(flywheelControllerKP, flywheelControllerKI, flywheelControllerKD);
    flywheelMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    SmartDashboard.putNumber("Flywheel kP", 0.0);
    SmartDashboard.putNumber("Flywheel kI", 0.0);
    SmartDashboard.putNumber("Flywheel kD", 0.0);
  }

  public void move(double speed) {
    flywheelMotor.set(ControlMode.PercentOutput, flywheelController.calculate(getVelocity(), speed));
  }
  public double getVelocity()
  {
    return flywheelMotor.getSelectedSensorVelocity();
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
  }
}
