// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Buttons;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CargoPlacementCommand extends CommandBase {
  private final LimelightSubsystem m_limelightSubsystem;
  private final Buttons m_Buttons;
  private final String m_buttonPressed;
  private Swerve m_driveSubsystem;

  private double range;
  private final double GOAL_RANGE_METERS = Units.feetToMeters(1);

  // PID constants should be tuned per robot
  private final double LINEAR_P = 0.1;
  private final double LINEAR_D = 0.0;
  private PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);
  
  

  private final double ANGULAR_P = 0.1;
  private double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  /** Creates a new CargoPlacementCommand. */
  public CargoPlacementCommand(Swerve driveSubsystem, LimelightSubsystem limelightSubsystem, Buttons buttons,
      String buttonPressed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_limelightSubsystem = limelightSubsystem;
    m_Buttons = buttons;
    m_buttonPressed = buttonPressed;
    m_driveSubsystem = driveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // TODO: tune apriltag 
    float Kp = -0.1f;
    float min_command = 0.05f;

    double forwardSpeed;
    double rotationSpeed;

    double tx = m_limelightSubsystem.getTx();
    double heading_error = -tx;
    double rotation_adjust = 0.0;

    // might be needed?????? might not be needed??????
    // if (tx > 1.0)
    // {
    //   rotation_adjust = Kp*heading_error - min_command;
    // }
    // else if (tx < 1.0)
    // {
    //   rotation_adjust = Kp*heading_error + min_command;
    // }

    double[] botpose = m_limelightSubsystem.getBotpose();
    double x = botpose[0];
    double y = botpose[1];
    double yaw = botpose[5];

    m_driveSubsystem.drive(
        new Translation2d(x, y).times(Constants.Swerve.maxSpeed), 
        yaw * Constants.Swerve.maxAngularVelocity, 
        false, 
        true
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
