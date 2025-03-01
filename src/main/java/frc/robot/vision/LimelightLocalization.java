package frc.robot.vision;
import java.util.List;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;


public class LimelightLocalization{
  public boolean rejectLeftData;
  public boolean rejectRightData;
  public boolean rejectMiddleData;
  public boolean disableLeft;
  public boolean disableRight;
  public boolean disableMiddle;
  public double limelightTXMiddle;
  public double limelightTAMiddle;
  public double limelightTXRight;
  public double limelightTARight;
  public double limelightTXLeft;
  public double limelightTALeft;
  public int limelightTagIDMiddle;
  public int limelightTagIDRight;


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
    new Pose2d(1.04, 7.11, Rotation2d.fromDegrees(-50)), // tag 13 CS
    new Pose2d(1.05, 0.91, Rotation2d.fromDegrees(50)) // tag 12 CS
    // new Pose2d(0.58, 7.13, Rotation2d.fromDegrees(-50)), // Left CS
    // new Pose2d(0.66, 1.1, Rotation2d.fromDegrees(50)) // Right CS
  };
  public Pose2d[] coralStationPosesRed = {
    new Pose2d(16.40, 0.99, Rotation2d.fromDegrees(130)), // tag 1 CS
    new Pose2d(16.28, 7.58, Rotation2d.fromDegrees(-130)) // tag 2 CS
  };
  public static List<Integer> coralStationTags = List.of(
    1, 2, 12, 13
  );
  public LimelightLocalization() {
    LimelightHelpers.setPipelineIndex("limelight-left", 0);
    LimelightHelpers.setPipelineIndex("limelight-right", 0);
    LimelightHelpers.setPipelineIndex("limelight-middle", 0);
  }
  public Pose2d[] getCoralStationPoses() {
    return Robot.alliance.get() == Alliance.Red ? coralStationPosesRed : coralStationPosesBlue;
  }

  public AlignmentState getReefAlignmentState(){

    double tolerance = 5.5;

    if ((Math.abs(limelightTXRight + 19.67) < tolerance && limelightTARight > 14.8) || (Math.abs(limelightTXLeft - 16.70) < tolerance && limelightTALeft > 14.3)) {
      return AlignmentState.ALIGNED;
    }
    else {
      return AlignmentState.NOT_ALIGNED;
    }

  }
  public void collectInputs(){
    limelightTXMiddle = LimelightHelpers.getTX("limelight-middle");
    limelightTAMiddle = LimelightHelpers.getTA("limelight-middle");
    limelightTXRight = LimelightHelpers.getTX("limelight-right");
    limelightTARight = LimelightHelpers.getTA("limelight-right");
    limelightTXLeft = LimelightHelpers.getTX("limelight-left");
    limelightTALeft= LimelightHelpers.getTA("limelight-left");
    limelightTagIDMiddle = (int)LimelightHelpers.getFiducialID("limelight-middle");
    limelightTagIDRight = (int)LimelightHelpers.getFiducialID("limelight-right");
  }
  
  public double getCoralStationAngleFromTag() {
    switch (limelightTagIDMiddle) {
      case 13:
        return -50;
      case 12:
        return 50;
      case 2:
        return -130;
      case 1:
        return 130;
      default:
        return CommandSwerveDrivetrain.getInstance().getState().Pose.getRotation().getDegrees();
    }
  }

  public double getReefAngleFromTag() {
    switch (limelightTagIDRight) {
      case 6:
        return 120;
      case 7:
        return 180;
      case 8:
        return -120;
      case 9:
        return -60;
      case 10:
        return 0;
      case 11:
        return 60;
      case 17:
        return 60;
      case 18:
        return 0;
      case 19:
        return -60;
      case 20:
        return -120;
      case 21:
        return 180;
      case 22:
        return 120;
      default:
      return CommandSwerveDrivetrain.getInstance().getState().Pose.getRotation().getDegrees();
    }
  }

  public AlignmentState getCoralStationAlignmentState(boolean isAuto){
    double tolerance = isAuto ? 2.25 : 3;

    if (Math.abs(limelightTXMiddle + 2.2) < tolerance && limelightTAMiddle > 3.9) {
      return AlignmentState.ALIGNED;
    }
    else{
      return AlignmentState.NOT_ALIGNED;
    }
  }

  public void update(){
    rejectLeftData = false;
    rejectRightData = false;
    rejectMiddleData = false;
    
    LimelightHelpers.SetRobotOrientation("limelight-left", CommandSwerveDrivetrain.getInstance().getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-right", CommandSwerveDrivetrain.getInstance().getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-middle", CommandSwerveDrivetrain.getInstance().getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2l = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
    LimelightHelpers.PoseEstimate mt2r = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");
    LimelightHelpers.PoseEstimate mt2m = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-middle");
    // if(CommandSwerveDrivetrain.getInstance().isMoving() && DriverStation.isTeleop()) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
    // {
    //   rejectLeftData = true;
    //   rejectRightData = true;
    //   rejectMiddleData = true;
    // }
    if(mt2m == null || mt2m.tagCount == 0 || disableMiddle)
    {
      rejectMiddleData = true;
    }
    if(mt2r == null || mt2r.tagCount == 0 || disableRight)
    {
      rejectRightData = true;
    }
    if(mt2l == null || mt2l.tagCount == 0 || disableLeft)
    {
      rejectLeftData = true;
    }
    if(!rejectRightData)
    {

      CommandSwerveDrivetrain.getInstance().addVisionMeasurement(
          mt2r.pose,
          Utils.fpgaToCurrentTime(mt2r.timestampSeconds),
          VecBuilder.fill(0.75,0.75,9999999));
      SmartDashboard.putNumber("mt2r", mt2r.timestampSeconds);

    }
    if(!rejectLeftData)
    {

      CommandSwerveDrivetrain.getInstance().addVisionMeasurement(
          mt2l.pose,
          Utils.fpgaToCurrentTime(mt2l.timestampSeconds),
          VecBuilder.fill(0.75, 0.75,9999999));
          SmartDashboard.putNumber("mt2l", mt2l.timestampSeconds);

    }
    if(!rejectMiddleData)
    {

      CommandSwerveDrivetrain.getInstance().addVisionMeasurement(
          mt2m.pose,
          Utils.fpgaToCurrentTime(mt2m.timestampSeconds),
          VecBuilder.fill(0.05, 0.05,9999999));
          SmartDashboard.putNumber("mt2m", mt2m.timestampSeconds);

    }
  }

  public void updateLeftCamera() {

  }

  private static LimelightLocalization instance;

    public static LimelightLocalization getInstance(){
       if (instance == null) instance = new LimelightLocalization(); // Make sure there is an instance (this will only run once)
        return instance;
    }

}
