package frc.robot.vision;
import java.util.List;

import com.ctre.phoenix6.Utils;

import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.FieldConstants;

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
  public int limelightTagIDLeft;
  public Pose2d[] branchPosesBlue = FieldConstants.getInstance().branchPosesBlue;
  public Pose2d[] branchPosesRed = FieldConstants.getInstance().branchPosesRed;
  public Pose2d[] coralStationPosesBlue = FieldConstants.getInstance().coralStationPosesBlue;
  public Pose2d[] coralStationPosesRed = FieldConstants.getInstance().coralStationPosesRed;
  
  public static List<Integer> coralStationTags = List.of(
    1, 2, 12, 13
  );

  public static List<Integer> reefTags = List.of(
  6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22
  );

  public LimelightLocalization() {
    LimelightHelpers.setPipelineIndex("limelight-left", 0);
    LimelightHelpers.setPipelineIndex("limelight-right", 0);
    LimelightHelpers.setPipelineIndex("limelight-middle", 0);
  }


  public AlignmentState getReefAlignmentState(){
    double tolerance = 5.5;

    if ((Math.abs(limelightTXRight + 19.67) < tolerance && limelightTARight > 14.6) || (Math.abs(limelightTXLeft - 16.70) < tolerance && limelightTALeft > 14.3)) {
      return AlignmentState.ALIGNED;
    } else if( !(limelightTARight > 14.6) || !(limelightTALeft > 14.3)){
      return AlignmentState.NOT_ALIGNED_FORWARD;
    } else {
      return AlignmentState.NOT_ALIGNED;
    }

  }

  public AlignmentState getBargeAlignmentState(){

    if ((limelightTAMiddle > 6)) {
      return AlignmentState.ALIGNED;
    }
    else {
      return AlignmentState.NOT_ALIGNED;
    }

  }

  public AlignmentState getCoralStationAlignmentState(boolean isAuto){
    double tolerance = isAuto ? 1.75 : 3;

    if (Math.abs(limelightTXMiddle + 2.2) < tolerance && limelightTAMiddle > 3.9) {
      return AlignmentState.ALIGNED;
    } else if( !(limelightTAMiddle > 3.9)) {
      return AlignmentState.NOT_ALIGNED_FORWARD;
    } else {
      return AlignmentState.NOT_ALIGNED;
    }
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
    switch (limelightTagIDRight){
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

  

  public Pose2d getAdjustedRobotPose() {
    Pose2d field_to_branch = FieldConstants.getInstance().getNearestBranch();
    return getAdjustedRobotPose(field_to_branch);
  }

  public Pose2d getAdjustedRobotPose(Pose2d branchPose) {
    Pose2d field_to_branch = branchPose;
    Pose2d branch_to_robot = new Pose2d(-0.5, 0, Rotation2d.kZero);
    return field_to_branch.plus(branch_to_robot.minus(new Pose2d()));
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
    limelightTagIDLeft = (int)LimelightHelpers.getFiducialID("limelight-left");
     DogLog.log("LimelightLocalization/Middle Limelight TX", limelightTXMiddle);
     DogLog.log("LimelightLocalization/Middle Limelight TA", limelightTAMiddle);
     DogLog.log("LimelightLocalization/Right Limelight TX", limelightTXRight);
     DogLog.log("LimelightLocalization/Right Limelight TA", limelightTARight);
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
          VecBuilder.fill(0.05,0.75,9999999));
      SmartDashboard.putNumber("mt2r", mt2r.timestampSeconds);

    }

    if(!rejectLeftData)
    {
      CommandSwerveDrivetrain.getInstance().addVisionMeasurement(
          mt2l.pose,
          Utils.fpgaToCurrentTime(mt2l.timestampSeconds),
          VecBuilder.fill(0.05, 0.05,9999999));
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
