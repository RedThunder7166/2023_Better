// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  private final NetworkTable m_cameraTable;
  private final NetworkTableEntry tx;
  private final NetworkTableEntry ty;
  private final NetworkTableEntry ta;
  private final NetworkTableEntry botpose;

  public LimelightSubsystem() {
    m_cameraTable = NetworkTableInstance.getDefault().getTable("limelight");
    tx = m_cameraTable.getEntry("tx");
    ty = m_cameraTable.getEntry("ty");
    ta = m_cameraTable.getEntry("ta");
    botpose = m_cameraTable.getEntry("botpose_targetspace");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("LimelightX", getTx());
    SmartDashboard.putNumber("LimelightY", getTy());
    SmartDashboard.putNumber("LimelightArea", getTa());
  }

  public double getTx(){
    return tx.getDouble(0.0);
  }
  public double getTy(){
    return ty.getDouble(0.0);
  }
  public double getTa(){
    return ta.getDouble(0.0);
  }
  public double[] getBotpose() {
    return botpose.getDoubleArray(new double[6]);
  }
}
