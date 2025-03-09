package frc.robot;

import java.util.List;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.elbow.ElbowSubsystem;
import frc.robot.vision.LimelightLocalization;

public final class FieldConstants {
  public Pose2d closestBranch;
  public Pose2d closestCoralStation;
  
  public Pose2d[] branchPosesBlue = {
    new Pose2d(3.96, 4.62, Rotation2d.fromDegrees(-60)), //L
    new Pose2d(4.24, 4.78, Rotation2d.fromDegrees(-60)), //K
    new Pose2d(4.74, 4.78, Rotation2d.fromDegrees(-120)), //J
    new Pose2d(5.02, 4.62, Rotation2d.fromDegrees(-120)), //I
    new Pose2d(5.27, 4.19, Rotation2d.fromDegrees(180)), //H
    new Pose2d(5.27, 3.86, Rotation2d.fromDegrees(180)), //G
    new Pose2d(5.02, 3.43, Rotation2d.fromDegrees(120)), //F
    new Pose2d(4.74, 3.27, Rotation2d.fromDegrees(120)), //E
    new Pose2d(4.24, 3.27, Rotation2d.fromDegrees(60)), //D
    new Pose2d(3.96, 3.43, Rotation2d.fromDegrees(60)), //C
    new Pose2d(3.71, 3.86, Rotation2d.fromDegrees(0)), //B
    new Pose2d(3.71, 4.19, Rotation2d.fromDegrees(0)), //A
  };
  public Pose2d[] branchPosesRed = {
    new Pose2d(13.59, 3.43, Rotation2d.fromDegrees(-60)), //L
    new Pose2d(13.31, 3.27, Rotation2d.fromDegrees(-60)), //K
    new Pose2d(12.81, 3.27, Rotation2d.fromDegrees(-120)), //J
    new Pose2d(12.53, 3.43, Rotation2d.fromDegrees(-120)), //I
    new Pose2d(12.29, 3.86, Rotation2d.fromDegrees(180)), //H
    new Pose2d(12.29, 4.19, Rotation2d.fromDegrees(180)), //G
    new Pose2d(12.53, 4.62, Rotation2d.fromDegrees(120)), //F
    new Pose2d(12.81, 4.78, Rotation2d.fromDegrees(120)), //E
    new Pose2d(13.31, 4.78, Rotation2d.fromDegrees(60)), //D
    new Pose2d(13.59, 4.62, Rotation2d.fromDegrees(60)), //C
    new Pose2d(13.84, 4.19, Rotation2d.fromDegrees(0)), //B
    new Pose2d(13.84, 3.86, Rotation2d.fromDegrees(0)), //A
  };
  public Pose2d[] coralStationPosesBlue = {
    new Pose2d(1.235, 7.11, Rotation2d.fromDegrees(-50)), // tag 13 CS
    new Pose2d(1.05, 0.91, Rotation2d.fromDegrees(50)) // tag 12 CS
  };
  public Pose2d[] coralStationPosesRed = {
    new Pose2d(16.40, 0.99, Rotation2d.fromDegrees(130)), // tag 1 CS
    new Pose2d(16.28, 7.58, Rotation2d.fromDegrees(-130)) // tag 2 CS
  };

  public Pose2d[] getCoralStationPoses() {
    return Robot.alliance.get() == Alliance.Red ? coralStationPosesRed : coralStationPosesBlue;
  }
  public Pose2d[] getReefPoses(){
    // return branchPoses;
    return Robot.alliance.get() == Alliance.Red ? branchPosesRed : branchPosesBlue;
  }

  public Pose2d getNearestBranch() {
      closestBranch = DrivetrainSubsystem.getInstance().drivetrain.getState().Pose.nearest(List.of(FieldConstants.getInstance().getReefPoses()));
      return closestBranch;
  }

  public Pose2d getNearestCoralStation() {
      closestCoralStation = DrivetrainSubsystem.getInstance().drivetrain.getState().Pose.nearest(List.of(FieldConstants.getInstance().getCoralStationPoses()));
      return closestCoralStation;
  }

  public void logBranches() {
    int i = 0;
    for (Pose2d pose : branchPosesBlue) {
      i++;
      DogLog.log("FieldConstants/branch pose " + i, pose);
      Pose2d offsetPose = LimelightLocalization.getInstance().getAdjustedRobotPose(pose);
      DogLog.log("FieldConstants/branch pose offset " + i, offsetPose);

    }
  }

  private static FieldConstants instance;
  
  public static FieldConstants getInstance() {
      if (instance == null) instance = new FieldConstants(); // Make sure there is an instance (this will only run once)
      return instance;
  }
}
