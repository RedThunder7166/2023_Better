// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PnuematicSubsystem;
import frc.robot.subsystems.theCLAAAWWW;

public class ClawCommand extends CommandBase {
  /** Creates a new ClawCommand. */
  private final theCLAAAWWW m_Claaawww;
  private final PnuematicSubsystem m_PnuematicSubsystem;
  public ClawCommand(theCLAAAWWW claw, PnuematicSubsystem p_hub) {
    m_Claaawww = claw;
    m_PnuematicSubsystem = p_hub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Claaawww);
    addRequirements(p_hub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    







  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
