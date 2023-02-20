// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PnuematicSubsystem extends SubsystemBase {
  /** Creates a new PnuematicSubsystem. */
  private PneumaticHub p_Hub; //this was evan not joey, dont blame me for this later pls (check C:\Users\lhsro\Documents\a\a\a\a\a\Coconut.jpg for more info)
 // private Solenoid clawSolenoid;


 public PnuematicSubsystem() {
   // p_Hub = new PneumaticHub(Constants.PneumaticsConstants.PneumaticsHubID); //  please buy us a new yyyyyyyyyyyy key
   // clawSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.PneumaticsConstants.ClawSolenoidID);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void toggleClawSolenoid() {
   // clawSolenoid.toggle();
  }

}
