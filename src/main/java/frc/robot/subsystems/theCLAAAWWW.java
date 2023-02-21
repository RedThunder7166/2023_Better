// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

public class theCLAAAWWW extends SubsystemBase {
  /** Creates a new theCLAAAWWW. */
  private WPI_TalonFX m_wristMotor;
  private WPI_TalonFX m_armMotorLeft;
  private WPI_TalonFX m_armMotorRight;

  private Buttons m_Buttons = new Buttons();
 // private XboxController xboxController = new XboxController(3);
  private enum ClawState {
    LOADING, LOW, MEDIUM, HIGH
  }


    //TODO Dont forget that EVERY thing needs a god forbidden PID. 

  ClawState clawState;

  ArmFeedforward armFeedForward = new ArmFeedforward(.21469, 0.43923, 2.0662);

  /**
   * also the arm.
   */

  private double wristAngle = 0;
  private double armAngle = 0;

  public theCLAAAWWW() {

    m_wristMotor = new WPI_TalonFX(Constants.Clawstants.ClawMotorWristID);
    m_armMotorLeft = new WPI_TalonFX(Constants.Clawstants.ClawMotorLeftID);
    m_armMotorRight = new WPI_TalonFX(Constants.Clawstants.ClawMotorRightID);

    m_armMotorLeft.config_kP(0, 0.2);
    m_armMotorLeft.config_kI(0, 0);
    m_armMotorLeft.config_kD(0, .1);
    m_armMotorLeft.configClosedLoopPeakOutput(0, .2);

    m_wristMotor.config_kP(0, 0.2);
    m_wristMotor.config_kI(0, 0);
    m_wristMotor.config_kD(0, .1);
    m_wristMotor.configClosedLoopPeakOutput(0, .2);

    /*
     * Left Motor - Leader
     * Right Motor - Follower - NEVER SET OR READ TO/FROM RIGHT MOTOR, DO ALL
     * CALCULATIONS BASED ON LEFT MOTOR
     */
    m_armMotorLeft.setInverted(true);
    m_wristMotor.setInverted(true);
    m_armMotorRight.follow(m_armMotorLeft);
    m_armMotorRight.setInverted(InvertType.OpposeMaster);

    clawState = ClawState.LOADING;

    m_armMotorLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    //m_wristMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    m_wristMotor.setNeutralMode(NeutralMode.Coast);
    m_armMotorLeft.setNeutralMode(NeutralMode.Brake);
    m_armMotorRight.setNeutralMode(NeutralMode.Brake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (m_Buttons.isPressed(1)) {
      clawState = ClawState.LOADING;
    } else if (m_Buttons.isPressed(2)) {
      clawState = ClawState.LOW;
    }

    switch (clawState) {
      case LOADING:
        wristAngle = 0;
        armAngle = 30;
        break;

      case LOW:
        wristAngle = 90;
        armAngle = 90;
        break;

      case HIGH:
        break;

      case MEDIUM:
        break;

    }

    // IMPORTANT - DO NOT DELETE
    // setArmMotorsAngle(armAngle);

    m_armMotorLeft.configMotionAcceleration(6000);
    m_armMotorLeft.configMotionCruiseVelocity(7000);

    m_wristMotor.configMotionAcceleration(6000);
    m_wristMotor.configMotionCruiseVelocity(7000);

    //m_wristMotor.set(ControlMode.Position, 30000);
     m_wristMotor.set( // changed from armMotorLeft
         ControlMode.MotionMagic,
         degreesToEncoderUnits(wristAngle, Constants.Clawstants.ClawWristGearRatio), // changed from armAngle
         DemandType.ArbitraryFeedForward,
         .03 * java.lang.Math.cos(encoderUnitsToDegrees(Math.toRadians(m_wristMotor.getSelectedSensorPosition()), Constants.Clawstants.ClawWristGearRatio)));

    m_armMotorLeft.set( 
        ControlMode.MotionMagic,
        degreesToEncoderUnits(armAngle, Constants.Clawstants.ClawArmGearRatio), // changed from armAngle
        DemandType.ArbitraryFeedForward,
        .03 * java.lang.Math
            .cos(encoderUnitsToDegrees(Math.toRadians(m_armMotorLeft.getSelectedSensorPosition()), Constants.Clawstants.ClawArmGearRatio)));

    SmartDashboard.putNumber("ArmMotorLeft Sensor Position", getLeftSensorPosition());
    SmartDashboard.putNumber("ArmMotorRight Sensor Position", getRightSensorPosition());
    SmartDashboard.putString("ClawState", clawState.name());

    SmartDashboard.putNumber("Arm Angle", encoderUnitsToDegrees(m_armMotorLeft.getSelectedSensorPosition(), Constants.Clawstants.ClawArmGearRatio));
    SmartDashboard.putNumber("Wrist Angle", encoderUnitsToDegrees(m_wristMotor.getSelectedSensorPosition(), Constants.Clawstants.ClawWristGearRatio));

    SmartDashboard.putNumber("Arm Encoder", m_armMotorLeft.getSelectedSensorPosition());
    SmartDashboard.putNumber("Wrist Encoder", m_wristMotor.getSelectedSensorPosition());

  }


  

  public double degreesToEncoderUnits(double angle, double gearRatio) {
    // double gearRatio = 30; //changed from 144
    double encoderUnitsPerMotorRotation = 2048;
    double encoderUnitsPerArmRotation = gearRatio * encoderUnitsPerMotorRotation; // 294,912

    return angle * encoderUnitsPerArmRotation / 360; // desired angle * 819.2

  }

  public double encoderUnitsToDegrees(double encoderUnits, double gearRatio) {
    // double gearRatio = 144;
    double encoderUnitsPerMotorRotation = 2048;
    double encoderUnitsPerArmRotation = gearRatio * encoderUnitsPerMotorRotation; // 294,912

    return (encoderUnits * 360) / encoderUnitsPerArmRotation;
  }

  // public void setArmMotorsAngle(double armAngle){
  // m_armMotorLeft.set(ControlMode.MotionMagic, degreesToEncoderUnits(armAngle),
  // DemandType.ArbitraryFeedForward, 0.07);

  
    // m_armMotorRight.set(ControlMode.Follower, degreesToEncoderUnits(-armAngle));
  
  // public void setWristMotorPercent(double percent) {
  // m_wristMotor.set(ControlMode., percent); // setting the wrist motor is a Work
  // In Progress.
  // }

  public void setArmOffsets() {
    m_armMotorLeft.setSelectedSensorPosition(0);
    // m_armMotorRight.setSelectedSensorPosition(0);

  }

  public void setWristOffsets() {
    m_wristMotor.setSelectedSensorPosition(0);
    // m_armMotorRight.setSelectedSensorPosition(0);

  }

  public double getLeftSensorPosition() {
    // return
    // m_armMotorLeft.getSensorCollection().getIntegratedSensorAbsolutePosition();
    return m_armMotorLeft.getSelectedSensorPosition();
  }

  public double getWristSensorPosition(){

    return m_wristMotor.getSelectedSensorPosition();

  }

  public double getRightSensorPosition() { // left and right should be synced, so this method is absolete
    // return
    // m_armMotorRight.getSensorCollection().getIntegratedSensorAbsolutePosition();
    return m_armMotorRight.getSelectedSensorPosition();
  }

 
  }
