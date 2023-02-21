// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Arm;
import frc.robot.Wrist;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

public class theCLAAAWWW extends SubsystemBase {
  /** Creates a new theCLAAAWWW. */
  private Buttons m_Buttons = new Buttons();

  public enum ClawState {
  
    LOADING, LOW, MEDIUM, HIGH
  }

  ClawState clawState;
  ClawState previousState;
  Arm arm = Arm.getInstance();
  Wrist wrist = Wrist.getInstance();

  // TODO Dont forget that EVERY thing needs a god forbidden PID.

  // ArmFeedforward armFeedForward = new ArmFeedforward(.21469, 0.43923, 2.0662);

  public theCLAAAWWW() {

    clawState = ClawState.LOADING;



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    previousState = clawState;

    if (m_Buttons.isPressed(1)) {
      clawState = ClawState.LOADING;
    } else if (m_Buttons.isPressed(2)) {
      clawState = ClawState.LOW;
    }
      else if (m_Buttons.isPressed(3)){
        clawState = ClawState.HIGH;
      }

    //Check to see if state has changed before moving the arm or claw.
    if (clawState != previousState) {
      switch (clawState) {
        case LOADING:
          wrist.setAngle(0);
          arm.setAngle(30);
          break;
        case LOW:
          wrist.setAngle(90);
          arm.setAngle(90);
          break;
        case HIGH:
          break;
        case MEDIUM:
          break;
      }

    }

    // IMPORTANT - DO NOT DELETE
    // setArmMotorsAngle(armAngle);
    
    SmartDashboard.putString("ClawState", clawState.name());

    SmartDashboard.putNumber("Arm Angle", arm.getAngle());
    SmartDashboard.putNumber("Wrist Angle", arm.getAngle());


    SmartDashboard.putNumber("Arm Encoder", arm.getRawEncoderUnits());
    SmartDashboard.putNumber("Wrist Encoder", wrist.getRawEncoderUnits());

  }

}
