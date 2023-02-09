// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  
  private PhotonCamera camera;
  private Transform3d robotToCam;
  private AprilTagFieldLayout aprilTagFieldLayout;
  private PhotonPoseEstimator robotPoseEstimator;
  // private Buttons buttons;

  public VisionSubsystem() {
    camera = new PhotonCamera("Limelight");
    
    // Set camera's position relative to the center of the robot
    robotToCam = new Transform3d(new Translation3d(0.305, 0.0889, 0), new Rotation3d(0, 0, 0));


    //JSON file available at https://github.com/wpilibsuite/allwpilib/tree/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag
    try {
      aprilTagFieldLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/2023AprilTagMap.json");
    } catch (IOException e) {
      String message = "Unable to open April Tag field layout JSON file. Ensure that it is in the deploy folder. Exception: "
          + e;
      SmartDashboard.putString("ERROR", message);
    }

    var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    camList.add(new Pair<PhotonCamera, Transform3d>(camera, robotToCam));
    robotPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, camera,
        robotToCam);
  }

  public double getRange() {
    var result = camera.getLatestResult();
    System.out.println("GOT PITCH: " + result.getBestTarget().getPitch());
    double range = PhotonUtils.calculateDistanceToTargetMeters(
        VisionConstants.CAMERA_HEIGHT_METERS,
        VisionConstants.TARGET_HEIGHT_METERS,
        VisionConstants.CAMERA_PITCH_RADIANS,
        Units.degreesToRadians(result.getBestTarget().getPitch()));
    return range;
  }

  public boolean hasTargets() {
    return camera.getLatestResult().hasTargets();
  }

  public PhotonTrackedTarget getTarget() {
    return camera.getLatestResult().getBestTarget();
  }

  public PhotonPipelineResult getLatestResult() {
    return camera.getLatestResult();
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("BUTTON PRESSING", -1);
    // int pressed = buttons.getPressedId();

    // if (/* target != null && */ pressed != 0) {
    // // double range = getRange();
    // switch (pressed) {
    // case 1:
    // // SmartDashboard.putBoolean("PRESSING ONE WITH TARGET", true);
    // SmartDashboard.putNumber("BUTTON PRESSING", 1);
    // PathPlannerTrajectory traj = PathPlanner.loadPath("New Path", new
    // PathConstraints(2, 2));
    // m_drive.followTrajectoryCommand(traj, true);
    // // System.out.println("PRESSING ONE");
    // break;
    // case 2:
    // // System.out.println("PRESSING TWO");
    // // SmartDashboard.putBoolean("PRESSING TWO WITH TARGET", true);
    // SmartDashboard.putNumber("BUTTON PRESSING", 2);
    // break;
    // case 3:
    // SmartDashboard.putNumber("BUTTON PRESSING", 3);
    // break;
    // case 4:
    // SmartDashboard.putNumber("BUTTON PRESSING", 4);
    // break;
    // case 5:
    // SmartDashboard.putNumber("BUTTON PRESSING", 5);
    // break;
    // case 6:
    // SmartDashboard.putNumber("BUTTON PRESSING", 6);
    // break;
    // case 7:
    // SmartDashboard.putNumber("BUTTON PRESSING", 7);
    // break;
    // case 8:
    // SmartDashboard.putNumber("BUTTON PRESSING", 8);
    // break;
    // case 9:
    // SmartDashboard.putNumber("BUTTON PRESSING", 9);
    // break;
    // case 10:
    // SmartDashboard.putNumber("BUTTON PRESSING", 10);
    // break;
    // // default:
    // // SmartDashboard.putNumber("BUTTON PRESSING", -1);
    // }

    // }
    int ID = -1;
    if (hasTargets()) {
      PhotonTrackedTarget target = getTarget();
      ID = target.getFiducialId();
      SmartDashboard.putNumber("TARGET ID", (double) ID);
      SmartDashboard.putNumber("Skew", target.getSkew());
      SmartDashboard.putNumber("Pitch2", target.getPitch());
      SmartDashboard.putNumber("Yaw", target.getYaw());
    }
    // SmartDashboard.putBoolean("Do I Has Targets?", hasTargets());

  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    // robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    // robotPoseEstimator.setLastPose(prevEstimatedRobotPose);
    return robotPoseEstimator.update();
  }

}