package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.elbow.ElbowSubsystem;

public final class FieldConstants {
  public Pose2d[] branchPoses = {
    new Pose2d(5, 5.247, Rotation2d.fromDegrees(-120)), //J
    new Pose2d(5.285, 5.107, Rotation2d.fromDegrees(-120)), //I
    new Pose2d(5.843, 4.2, Rotation2d.fromDegrees(180)), //H
    new Pose2d(5.843, 3.841, Rotation2d.fromDegrees(180)), //G
    new Pose2d(5.295, 2.923, Rotation2d.fromDegrees(120)), //F
    new Pose2d(5.036, 2.784, Rotation2d.fromDegrees(120)), //E
    new Pose2d(3.949, 2.774, Rotation2d.fromDegrees(60)), //D
    new Pose2d(3.65, 2.943, Rotation2d.fromDegrees(60)), //C
    new Pose2d(3.171, 3.841, Rotation2d.fromDegrees(0)), //B
    new Pose2d(3.171, 4.170, Rotation2d.fromDegrees(0)), //A
    new Pose2d(3.719, 5.087, Rotation2d.fromDegrees(-60)), //L
    new Pose2d(4, 5.227, Rotation2d.fromDegrees(-60)), //K
  };
  public Pose2d[] coralStationPosesBlue = {
    new Pose2d(1.235, 7.11, Rotation2d.fromDegrees(-50)), // tag 13 CS
    new Pose2d(1.05, 0.91, Rotation2d.fromDegrees(50)) // tag 12 CS
  };
  public Pose2d[] coralStationPosesRed = {
    new Pose2d(16.40, 0.99, Rotation2d.fromDegrees(130)), // tag 1 CS
    new Pose2d(16.28, 7.58, Rotation2d.fromDegrees(-130)) // tag 2 CS
  };
  
  private static FieldConstants instance;
  
  public static FieldConstants getInstance() {
      if (instance == null) instance = new FieldConstants(); // Make sure there is an instance (this will only run once)
      return instance;
  }
}
