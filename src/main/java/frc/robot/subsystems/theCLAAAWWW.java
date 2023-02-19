// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;


public class theCLAAAWWW extends SubsystemBase {
  /** Creates a new theCLAAAWWW. */
  private CANCoder m_encoder;
  private WPI_TalonFX m_wristMotor;
  private WPI_TalonFX m_armMotorLeft;
  private WPI_TalonFX m_armMotorRight;

  private double armPercent = 0;
  private double wristPercent = 0;

  private enum ClawState {
    LOADING, LOW, MEDIUM, HIGH
  }

  private double wristPosition = 0;//guess what?  change later cause you did this wrong, no surpise there

  ClawState clawState;

  private double previousArmPosition;
  private double currentArmPosition;

  ArmFeedforward armFeedForward = new ArmFeedforward(.21469, 0.43923, 2.0662);
  
  

  /**
   * also the arm.
   */
  public theCLAAAWWW() {
  
    //m_encoder = new CANCoder(Constants.Clawstants.ClawEncoderID);
    m_wristMotor = new WPI_TalonFX(Constants.Clawstants.ClawMotorWristID);
    m_armMotorLeft = new WPI_TalonFX(Constants.Clawstants.ClawMotorLeftID);
    m_armMotorRight = new WPI_TalonFX(Constants.Clawstants.ClawMotorRightID);

    m_armMotorLeft.config_kP(0, .1);
    m_armMotorLeft.config_kI(0, 0);
    m_armMotorLeft.config_kD(0, 0);

    currentArmPosition = getRightSensorPosition();  
    clawState = ClawState.LOADING;

    m_armMotorLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    m_wristMotor.setNeutralMode(NeutralMode.Brake);
    m_armMotorLeft.setNeutralMode(NeutralMode.Brake);
    m_armMotorRight.setNeutralMode(NeutralMode.Brake);

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setArmMotorsPercent(armPercent);
    setWristMotorPercent(wristPercent);
    // setWristMotorsPercent(wristPercent);

    SmartDashboard.putNumber("Arm Angle", encoderUnitsToDegrees(m_armMotorLeft.getSelectedSensorPosition()));
    SmartDashboard.putNumber("Arm Encoder", m_armMotorLeft.getSelectedSensorPosition());

    previousArmPosition = currentArmPosition;
    currentArmPosition = getRightSensorPosition();
  
    boolean currentIsAboveLoadingToLow = currentArmPosition > Constants.Clawstants.ClawLoadingToLowPosition;
    boolean previousIsAboveLoadingToLow = previousArmPosition > Constants.Clawstants.ClawLoadingToLowPosition;
    if (currentIsAboveLoadingToLow && !previousIsAboveLoadingToLow){
      clawState = ClawState.LOW;
    } else if (!currentIsAboveLoadingToLow && previousIsAboveLoadingToLow){
      clawState = ClawState.LOADING;
    };
    
    switch(clawState){
      case LOADING:
        wristPosition = 0;
        break;

      case LOW:
        wristPosition = 3000;
        break;

      case HIGH:
        break;

      case MEDIUM:
        break;
      
    }
    

    SmartDashboard.putNumber("ArmMotorLeft Sensor Position", getLeftSensorPosition());
    SmartDashboard.putNumber("ArmMotorRight Sensor Position", getRightSensorPosition());
    SmartDashboard.putString("ClawState", clawState.name());
  }

  public double degreesToEncoderUnits(double angle){
    double gearRatio = 144;
    double encoderUnitsPerMotorRotation = 2048;
    double encoderUnitsPerArmRotation = gearRatio * encoderUnitsPerMotorRotation; //294,912

    return angle * encoderUnitsPerArmRotation/360; //desired angle * 819.2

  }

  public double encoderUnitsToDegrees(double encoderUnits){
    double gearRatio = 144;
    double encoderUnitsPerMotorRotation = 2048;
    double encoderUnitsPerArmRotation = gearRatio * encoderUnitsPerMotorRotation; //294,912

    return (encoderUnits * 360)/encoderUnitsPerArmRotation;
  }
 
  public void setArmMotorsPercent(double percent) {

    // m_armMotorLeft.set(ControlMode.Position,  );//Changed from .PercentOutput
    // m_armMotorRight.set(ControlMode.PercentOutput, -percent);
    
    m_armMotorLeft.set(ControlMode.PercentOutput, percent);
    m_armMotorRight.set(ControlMode.PercentOutput, -percent);
  }
  public void setWristMotorPercent(double wristPercent){
  
    m_wristMotor.set(ControlMode.PercentOutput, wristPercent);


   // m_wristMotor.set(ControlMode.PercentOutput, wristpercent);

  }
  // public void setWristMotorPercent(double percent) {
    // m_wristMotor.set(ControlMode., percent); // setting the wrist motor is a Work In Progress.
  // }

  public void armUp(){
    armPercent = -0.1;
  }
  public void armDown(){
    armPercent = 0.1;
  }
  public void armStop(){
    armPercent = 0;
  }
  public void wristLeft(){
    wristPercent = -0.2;
  }
  public void wristRight(){
    wristPercent = 0.2;
  }
  public void wristStop(){
    wristPercent = 0;
  }

  public void setArmOffsets() {
    m_armMotorLeft.setSelectedSensorPosition(0);
    m_armMotorRight.setSelectedSensorPosition(0);
    
  }

  public double getLeftSensorPosition() {
    //return m_armMotorLeft.getSensorCollection().getIntegratedSensorAbsolutePosition();
     return m_armMotorLeft.getSelectedSensorPosition();
  }
  public double getRightSensorPosition() { // left and right should be synced, so this method is absolete
    //return m_armMotorRight.getSensorCollection().getIntegratedSensorAbsolutePosition();
     return m_armMotorRight.getSelectedSensorPosition();
  }
}
