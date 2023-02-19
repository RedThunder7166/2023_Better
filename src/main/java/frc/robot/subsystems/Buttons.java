// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Buttons extends SubsystemBase {
  /** Creates a new Buttons. */
  GenericHID m_leonardoArduino = new GenericHID(5);

  String[] buttons = {
    "Node1",
    "Node2",
    "Node3",
    "Node4",
    "Node5",
    "Node6",
    "Node7",
    "Node8",
    "Node9",
    "Reset",
    "Stop",
    "WristUp",//might be automated
    "WristDown",//might be automated
    "HopperPlacement",
    "Solenoid"
  };

  public Buttons() {
  }
  public boolean isPressed(int id) {
    return m_leonardoArduino.getRawButton(id);
  }

  /**
   * Gets the ID of the button being pressed with the lowest ID.
   * @return <int> button ID
   */
  public int getPressedId() {
    for (int id = 0; id < buttons.length; id++) {
      // we add one to id because arrays start at 0 while the button board starts at 1
      if (isPressed(id + 1)) {
        return id + 1;
      }
    }
    return 0;
  }
  public String getPressedName(){
    int index = getPressedId() - 1;
    if (index < 0) {
      return "none";
    }
    return buttons[index];
  }

  // public double getPotentiometer(){
  //   return m_leonardoArduino.getRawAxis(Constants.Clawstants.PotentiometerID);
  // }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}