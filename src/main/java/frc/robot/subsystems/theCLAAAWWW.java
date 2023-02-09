// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;



public class theCLAAAWWW extends SubsystemBase {
  /** Creates a new theCLAAAWWW. */
  private CANCoder m_encoder;
  private TalonFX m_clawMotorWrist;
  private TalonFX m_clawMotorLeft;
  private TalonFX m_clawMotorRight;

  private double clawVoltage = 0;

  /**
   * also the arm.
   */
  public theCLAAAWWW() {
  
    //m_encoder = new CANCoder(Constants.Clawstants.ClawEncoderID);
    m_clawMotorWrist = new TalonFX(Constants.Clawstants.ClawMotorWristID);
    m_clawMotorLeft = new TalonFX(Constants.Clawstants.ClawMotorLeftID);
    m_clawMotorRight = new TalonFX(Constants.Clawstants.ClawMotorRightID);

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setLeftRight(clawVoltage);
  }

 
 
 
  public void setLeftRight(double voltage) {
    m_clawMotorLeft.set(ControlMode.PercentOutput, voltage);
    m_clawMotorRight.set(ControlMode.PercentOutput, -voltage);
  }
  public void up(){
    clawVoltage = 0.2;
  }
  public void down(){
    clawVoltage = -0.2;
  }

  public void stop(){
    clawVoltage = 0;
  }

  public double getLeftSensorPosition() {
    return m_clawMotorLeft.getSensorCollection().getIntegratedSensorPosition();
  }
  public double getRightSensorPosition() { // left and right should be synced, so this method is absolete
    return m_clawMotorRight.getSensorCollection().getIntegratedSensorPosition();
  }
}
