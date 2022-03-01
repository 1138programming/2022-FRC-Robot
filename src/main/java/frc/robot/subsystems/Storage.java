package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Storage extends SubsystemBase {
  //Defining motors
  private VictorSPX bottomStorageMotor;
  private VictorSPX topStorageMotor;
  //Defining encoders
  private DigitalInput ballSensorBottom;
  private DigitalInput ballSensorTop;

  private boolean isMoving = false;

  //Constructor is constructing
  public Storage(){
    //set the motor
    bottomStorageMotor = new VictorSPX(KBottomStorageVictor);
    topStorageMotor = new VictorSPX(KTopStorageVictor);
    //set the sensor to detect the motor
    ballSensorBottom = new DigitalInput(KBallSensorBottom);
    ballSensorTop = new DigitalInput(KBallSensorTop);
  }

  @Override
  public void periodic() {
  }

  //move function, to make top run bottom not run, (1,0), vice versa
  public void move(double topSpeed, double bottomSpeed) {
    topStorageMotor.set(VictorSPXControlMode.PercentOutput, topSpeed);
    bottomStorageMotor.set(VictorSPXControlMode.PercentOutput, bottomSpeed);
  }
  public void moveTop(double speed) {
    topStorageMotor.set(VictorSPXControlMode.PercentOutput, speed);
    isMoving = true;
  }
  public void moveBottom(double speed) {
    bottomStorageMotor.set(VictorSPXControlMode.PercentOutput, speed);
    isMoving = true;
  }

  public boolean getBallSensorTop(){
    return ballSensorTop.get();
  }
  public boolean getBallSensorBottom(){
    return ballSensorBottom.get();
  }
}