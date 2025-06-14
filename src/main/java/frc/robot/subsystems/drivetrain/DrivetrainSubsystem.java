package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;
import frc.robot.FmsSubsystem;
import frc.robot.Robot;
import frc.robot.StateMachine;
import frc.robot.commands.RobotManager;
import frc.robot.drivers.Xbox;
import frc.robot.vision.AlignmentState;
import frc.robot.vision.LimelightLocalization;
import frc.robot.vision.LimelightState;
import frc.robot.vision.LimelightSubsystem;

public class DrivetrainSubsystem extends StateMachine<DrivetrainState> {
  private final String name = getName();
  private  double MaxSpeed = TunerConstants.kSpeedAt12Volts;
  private ChassisSpeeds teleopSpeeds = new ChassisSpeeds();
  private ChassisSpeeds autoSpeeds = new ChassisSpeeds();
  private ChassisSpeeds reefAutoAlignSpeeds = new ChassisSpeeds();
  private ChassisSpeeds coralStationAutoAlignSpeeds = new ChassisSpeeds();
  public PIDController coralStationAutoAlignTX = new PIDController(0, 0, 0);
  public PIDController coralStationAutoAlignTA = new PIDController(0, 0, 0);
  public PIDController reefAutoAlignTX = new PIDController(0, 0, 0);
  public PIDController reefAutoAlignTA = new PIDController(0, 0, 0);
  private final double MaxAngularRate = Math.PI * 3.5;
  public final CommandSwerveDrivetrain drivetrain;
  private LimelightLocalization limelightLocalization = LimelightLocalization.getInstance();
  private double snapCoralStationAngle;
  public boolean crescendoModeEnabled;
  private double snapReefAngle;
  private double snapAlgaeAngle;
  private double snapBargeAngle;
  private int coralStationTag;
  private int reefTag;
  private Pose2d targetCoralStationPose;
  private Pose2d targetReefPose;
  private Pose2d targetAlgaePose;
  private SwerveDriveState drivetrainState = new SwerveDriveState();
  private double goalSnapAngle = 0;
  private double leftY;
  private double leftX;
  private double rightX;
  // private boolean hasAppliedOperatorPerspective = false;
  // private Rotation2d operatorPerspective;

  public final Pigeon2 drivetrainPigeon = CommandSwerveDrivetrain.getInstance().getPigeon2();

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
          // I want field-centric driving in open loop
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withDeadband(MaxSpeed * 0.03)
          .withRotationalDeadband(MaxAngularRate * 0.03);

    public final SwerveRequest.RobotCentric driveRobotRelative =
          new SwerveRequest.RobotCentric()
              // I want field-centric driving in open loop
              .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
              .withDeadband(MaxSpeed * 0.03)
              .withRotationalDeadband(MaxAngularRate * 0.03);

  private final SwerveRequest.FieldCentricFacingAngle driveToAngle =
      new SwerveRequest.FieldCentricFacingAngle()
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withDeadband(MaxSpeed * 0.03);

  public DrivetrainSubsystem() {
    super(DrivetrainState.TELEOP);
    crescendoModeEnabled = true;
    LimelightSubsystem.getInstance();
    drivetrain = CommandSwerveDrivetrain.getInstance();
    driveToAngle.HeadingController.setPID(6, 0, 0);
    driveToAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    driveToAngle.HeadingController.setTolerance(0.5);

    coralStationAutoAlignTA.setPID(0.3, 0, 0);
    coralStationAutoAlignTX.setPID(1.0 / 10.0, 0, 0);

    reefAutoAlignTA.setPID(0.1, 0, 0);
    reefAutoAlignTX.setPID(0.03, 0, 0);

    reefAutoAlignSpeeds = new ChassisSpeeds(0, 0, 0);
    coralStationAutoAlignSpeeds = new ChassisSpeeds(0, 0, 0);
  }

  public SwerveDriveState getDrivetrainState() {
    return drivetrainState;
  }

  @Override
  protected DrivetrainState getNextState(DrivetrainState currentState) {
    DrivetrainState nextState = currentState;
     switch (currentState) {
      case BARGE_ALIGN -> {
        switch (RobotManager.getInstance().getState()) {
          case IDLE, INVERTED_IDLE, PREPARE_IDLE, PREPARE_INVERTED_IDLE, PREPARE_INVERTED_FROM_IDLE, PREPARE_IDLE_FROM_INVERTED, PREPARE_REMOVE_ALGAE_HIGH, PREPARE_REMOVE_ALGAE_LOW, WAIT_REMOVE_ALGAE_HIGH, WAIT_REMOVE_ALGAE_LOW, REMOVE_ALGAE_HIGH, REMOVE_ALGAE_LOW-> {
            nextState = DrivetrainState.TELEOP;
          }

          default -> {}
        }
      }
      case AUTO_CORAL_STATION_ALIGN_1 -> {
        if (CommandSwerveDrivetrain.getInstance().isNear(targetCoralStationPose) || timeout(0.6)) {
          nextState = DrivetrainState.AUTO;
        }
      }
      case AUTO_CORAL_STATION_ALIGN_2 -> {
        if (LimelightLocalization.getInstance().getCoralStationAlignmentState(true) == AlignmentState.ALIGNED) {
          nextState = DriverStation.isAutonomous() ? DrivetrainState.AUTO : DrivetrainState.TELEOP;
        }
      }
      case AUTO_REEF_ALIGN_1 -> {
        if (CommandSwerveDrivetrain.getInstance().isNear(targetReefPose) || timeout(0.6)) {
          if (DriverStation.isAutonomous()) {
              
              nextState = DrivetrainState.AUTO;
          }
          else{
            nextState = DrivetrainState.TELEOP;
          }
        }
      }
      case AUTO_REEF_ALIGN_2 -> {
        if (LimelightLocalization.getInstance().getReefAlignmentState() == AlignmentState.ALIGNED) {
          nextState = DriverStation.isAutonomous() ? DrivetrainState.AUTO : DrivetrainState.TELEOP;
        }
      }
      case AUTO_ALGAE_ALIGN -> {
        if (CommandSwerveDrivetrain.getInstance().isNear(targetAlgaePose) || timeout(0.6)) {
          if (DriverStation.isAutonomous()) {
            nextState = DrivetrainState.AUTO;
          }
          else {
            nextState = DrivetrainState.TELEOP;
          }
        }
      }
      case TELEOP_CORAL_STATION_ALIGN, TELEOP_REEF_SNAP, TELEOP_REEF_ALIGN-> {
        switch (RobotManager.getInstance().getState()) {
          case IDLE, INVERTED_IDLE, PREPARE_IDLE, PREPARE_INVERTED_IDLE, PREPARE_INVERTED_FROM_IDLE, PREPARE_IDLE_FROM_INVERTED, PREPARE_REMOVE_ALGAE_LOW, PREPARE_REMOVE_ALGAE_HIGH, REMOVE_ALGAE_LOW, REMOVE_ALGAE_HIGH -> {
            nextState = DrivetrainState.TELEOP;
          }
          case PREPARE_L1, PREPARE_L2, PREPARE_L3, PREPARE_L4, WAIT_L1, WAIT_L2, WAIT_L3, WAIT_L4, SCORE_L1, SCORE_L2, SCORE_L3, SCORE_L4, CAPPED_L4-> {
            if (!RobotManager.getInstance().isHeightCapped && crescendoModeEnabled) {
              nextState = DrivetrainState.TELEOP_REEF_SNAP;
            } 
            if (RobotManager.getInstance().isHeightCapped) {
              nextState = DrivetrainState.TELEOP_REEF_ALIGN;
            } 
          }

          default -> {}
        }
      }
      case TELEOP-> {
        switch (RobotManager.getInstance().getState()) {
          case PREPARE_CORAL_STATION, PREPARE_INVERTED_CORAL_STATION, INVERTED_INTAKE_CORAL_STATION, INTAKE_CORAL_STATION-> {
            nextState = DrivetrainState.TELEOP_CORAL_STATION_ALIGN;
          }
          case PREPARE_L1, PREPARE_L2, PREPARE_L3, PREPARE_L4, WAIT_L1, WAIT_L2, WAIT_L3, WAIT_L4, SCORE_L1, SCORE_L2, SCORE_L3, SCORE_L4, CAPPED_L4-> {
            if (RobotManager.getInstance().isHeightCapped == true) {
              nextState = DrivetrainState.TELEOP_REEF_ALIGN;
            } 
          }
          case PREPARE_SCORE_ALGAE, SCORE_ALGAE_WAIT, SCORE_ALGAE, PRE_FRONT_SCORE_ALGAE, FRONT_SCORE_ALGAE_WAIT, FRONT_SCORE_ALGAE -> {
            if (crescendoModeEnabled){
              nextState = DrivetrainState.BARGE_ALIGN;
            }
        }
          default -> {}
        }
      }
      default -> {}
     }
    return nextState;
  }

  public void setAutoSpeeds(ChassisSpeeds speeds) {
    autoSpeeds = speeds;
  }
  
  public void setSnapToAngle(double angle) {
    goalSnapAngle = angle;
  }

  boolean isNotControlled(ChassisSpeeds speeds) {
    return Math.abs(speeds.vxMetersPerSecond) < 0.01 && Math.abs(speeds.vyMetersPerSecond) < 0.01 && Math.abs(speeds.omegaRadiansPerSecond) < 0.01;
  }

  @Override
  protected void collectInputs() {
    leftY = Robot.controls.driver.leftY();
    leftX = Robot.controls.driver.leftX();
    rightX = Robot.controls.driver.rightX();
  
    limelightLocalization.update();
    drivetrainState = drivetrain.currentState;
    teleopSpeeds = new ChassisSpeeds(
      -leftY * leftY * leftY * MaxSpeed, 
      -leftX * leftX * leftX * MaxSpeed, 
      rightX * rightX * rightX * MaxAngularRate);

    boolean isSlow = false;
    if (!RobotManager.getInstance().isHeightCapped) {
      teleopSpeeds = teleopSpeeds.div(2);
      isSlow = true;
    }

    if (getState() == DrivetrainState.AUTO_CORAL_STATION_ALIGN_1 || getState() == DrivetrainState.AUTO_CORAL_STATION_ALIGN_2) {
      targetCoralStationPose = LimelightLocalization.getInstance().getAdjustedCoralStationPose();
      snapCoralStationAngle = targetCoralStationPose.getRotation().getDegrees();
      coralStationTag = limelightLocalization.limelightTagIDMiddle;
      boolean isValidTag = LimelightLocalization.coralStationTags.contains(coralStationTag);
      if (isValidTag) {
        double coralStationSpeedX = -coralStationAutoAlignTA.calculate(limelightLocalization.limelightTAMiddle, 3.9);
        double coralStationSpeedY = coralStationAutoAlignTX.calculate(-limelightLocalization.limelightTXMiddle, -2.2);
        coralStationAutoAlignSpeeds = new ChassisSpeeds(coralStationSpeedX, coralStationSpeedY, 0);
      } else {
        coralStationAutoAlignSpeeds = new ChassisSpeeds();
      }
    }

    if (getState() == DrivetrainState.AUTO_REEF_ALIGN_1 || getState() == DrivetrainState.AUTO_REEF_ALIGN_2) {
      targetReefPose = LimelightLocalization.getInstance().getAdjustedBranchPose();
      snapReefAngle = targetReefPose.getRotation().getDegrees();
      reefTag = limelightLocalization.limelightTagIDLeft;
      boolean isValidTag = LimelightLocalization.reefTags.contains(reefTag);
      if (isValidTag) {
        double reefSpeedX = -reefAutoAlignTA.calculate(limelightLocalization.limelightTAMiddle + teleopSpeeds.vxMetersPerSecond, 14.6);
        double reefSpeedY = reefAutoAlignTX.calculate(-limelightLocalization.limelightTXMiddle + teleopSpeeds.vyMetersPerSecond, -19.7);
        reefAutoAlignSpeeds = new ChassisSpeeds(reefSpeedX, reefSpeedY, 0);
      } else {
        reefAutoAlignSpeeds = new ChassisSpeeds();
      }
    }
    if (getState() == DrivetrainState.TELEOP_REEF_SNAP || getState() == DrivetrainState.TELEOP_REEF_ALIGN) {
      targetReefPose = FieldConstants.getInstance().getNearestBranch();
      snapReefAngle = targetReefPose.getRotation().getDegrees();
    }

    if (getState() == DrivetrainState.AUTO_ALGAE_ALIGN) {
      targetAlgaePose = LimelightLocalization.getInstance().getAdjustedAlgaePose();
      snapAlgaeAngle = targetAlgaePose.getRotation().getDegrees();
    }

    // if (getState() == DrivetrainState.AUTO_REEF_ALIGN) {
    //   double reefSpeedX = reefAutoAlignTA.calculate(limelightLocalization.limelightTARight, 14);
    //   double reefSpeedY = reefAutoAlignTX.calculate(limelightLocalization.limelightTXRight, -14);
    //   reefAutoAlignSpeeds = new ChassisSpeeds(reefSpeedX, reefSpeedY, 0);
    // }
    snapReefAngle = LimelightLocalization.getInstance().getReefAngleFromTag();
    snapBargeAngle = LimelightLocalization.getInstance().getBargeSnapAngle();
    
    DogLog.log(name + "/isSlow", isSlow);
    DogLog.log(name + "/Reef Snap Angle", snapReefAngle);
    DogLog.log(name + "/Coral Station Snap Angle", snapCoralStationAngle);
    DogLog.log(name + "/teleopSpeeds", teleopSpeeds);
    DogLog.log(name + "/robot pose", CommandSwerveDrivetrain.getInstance().currentState.Pose);
    DogLog.log(name + "/coralStationTag", coralStationTag);
    DogLog.log(name + "/target reef pose", targetReefPose);
  }
  @Override
  public void periodic() {
    super.periodic();
    sendSwerveRequest(getState());
    CommandSwerveDrivetrain.getInstance().update();
  }

    @Override
    protected void afterTransition(DrivetrainState newState) {
        switch (newState) {
          case TELEOP -> {
           LimelightSubsystem.getInstance().setState(LimelightState.DRIVE);
          }
          case AUTO -> {
            LimelightSubsystem.getInstance().setState(LimelightState.AUTO);
          }
          case TELEOP_REEF_ALIGN -> {
            LimelightSubsystem.getInstance().setState(LimelightState.REEF);
          }
          case TELEOP_REEF_SNAP -> {
            LimelightSubsystem.getInstance().setState(LimelightState.REEF);
          }
          case TELEOP_CORAL_STATION_ALIGN -> {
            LimelightSubsystem.getInstance().setState(LimelightState.CORAL_STATION);
          }
          case AUTO_ALGAE_ALIGN -> {
            LimelightSubsystem.getInstance().setState(LimelightState.REEF);
          }
          case AUTO_REEF_ALIGN_1 -> {
            LimelightSubsystem.getInstance().setState(LimelightState.AUTO_REEF);
          }
          case AUTO_CORAL_STATION_ALIGN_1 -> {
            LimelightSubsystem.getInstance().setState(LimelightState.AUTO_CORAL_STATION);
          }
          case BARGE_ALIGN -> {
            LimelightSubsystem.getInstance().setState(LimelightState.BARGE_ALIGN);
          }
           default -> {}
        }
    }

    protected void sendSwerveRequest(DrivetrainState newState) {
      switch (newState) {
      case TELEOP -> {
        drivetrain.setControl(
          drive
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
          .withVelocityX(teleopSpeeds.vxMetersPerSecond)
          .withVelocityY(teleopSpeeds.vyMetersPerSecond)
          .withRotationalRate(teleopSpeeds.omegaRadiansPerSecond)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
      }
      case BARGE_ALIGN -> {
        drivetrain.setControl(
          driveToAngle
          .withVelocityX(teleopSpeeds.vxMetersPerSecond)
          .withVelocityY(teleopSpeeds.vyMetersPerSecond)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
          .withTargetDirection(Rotation2d.fromDegrees(snapBargeAngle)));
      }
      case TELEOP_CORAL_STATION_ALIGN -> {
        drivetrain.setControl(
          drive
          .withVelocityX(teleopSpeeds.vxMetersPerSecond)
          .withVelocityY(teleopSpeeds.vyMetersPerSecond)
          .withRotationalRate(teleopSpeeds.omegaRadiansPerSecond)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
      }

      case TELEOP_REEF_ALIGN -> {
          drivetrain.setControl(
            drive
            .withVelocityX(teleopSpeeds.vxMetersPerSecond)
            .withVelocityY(teleopSpeeds.vyMetersPerSecond)
            .withRotationalRate(teleopSpeeds.omegaRadiansPerSecond)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
        }

        case TELEOP_REEF_SNAP -> {
          drivetrain.setControl(
            driveToAngle
            .withVelocityX(teleopSpeeds.vxMetersPerSecond)
            .withVelocityY(teleopSpeeds.vyMetersPerSecond)
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
            .withTargetDirection(FmsSubsystem.isRedAlliance() ? targetReefPose.getRotation().plus(Rotation2d.fromDegrees(180)) : targetReefPose.getRotation()) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
        }

      case AUTO_CORAL_STATION_ALIGN_1 -> {
        ChassisSpeeds speeds = drivetrain.driveToPoseSpeeds(targetCoralStationPose);
        drivetrain.setControl(

          driveToAngle
            .withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withTargetDirection(targetCoralStationPose.getRotation())
        );
      }

      case AUTO_CORAL_STATION_ALIGN_2 -> {
        if (!MathUtil.isNear(snapCoralStationAngle, CommandSwerveDrivetrain.getInstance().currentState.Pose.getRotation().getDegrees(), 3)) {
          drivetrain.setControl(
            driveToAngle
              .withVelocityX(autoSpeeds.vxMetersPerSecond)
              .withVelocityY(autoSpeeds.vyMetersPerSecond)
              .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
              .withTargetDirection(Rotation2d.fromDegrees(snapCoralStationAngle)));
        } else {
          drivetrain.setControl(
            driveRobotRelative
              .withVelocityX(coralStationAutoAlignSpeeds.vxMetersPerSecond)
              .withVelocityY(coralStationAutoAlignSpeeds.vyMetersPerSecond)
              .withDriveRequestType(DriveRequestType.Velocity));
          }
        }
        
        case AUTO_ALGAE_ALIGN -> {
          ChassisSpeeds speeds = drivetrain.driveToPoseSpeeds(targetAlgaePose);
          drivetrain.setControl(
  
            driveToAngle
              .withVelocityX(speeds.vxMetersPerSecond)
              .withVelocityY(speeds.vyMetersPerSecond)
              .withDriveRequestType(DriveRequestType.Velocity)
              .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
              .withTargetDirection(targetAlgaePose.getRotation())
          );
        }

      case AUTO_REEF_ALIGN_1 -> {
          ChassisSpeeds speeds = drivetrain.driveToPoseSpeeds(targetReefPose);
          drivetrain.setControl(
            driveToAngle
              .withVelocityX(speeds.vxMetersPerSecond + teleopSpeeds.vxMetersPerSecond)
              .withVelocityY(speeds.vyMetersPerSecond + teleopSpeeds.vyMetersPerSecond)
              .withDriveRequestType(DriveRequestType.Velocity)
              .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
              .withTargetDirection(targetReefPose.getRotation())
          );
        }
        
      case AUTO_REEF_ALIGN_2 -> {
        if (!MathUtil.isNear(snapReefAngle, CommandSwerveDrivetrain.getInstance().currentState.Pose.getRotation().getDegrees(), 3)) {
          drivetrain.setControl(
            driveToAngle
              .withVelocityX(autoSpeeds.vxMetersPerSecond)
              .withVelocityY(autoSpeeds.vyMetersPerSecond)
              .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
              .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
              .withTargetDirection(Rotation2d.fromDegrees(snapReefAngle)));
        } else {
          drivetrain.setControl(
            driveRobotRelative
              .withVelocityX(reefAutoAlignSpeeds.vxMetersPerSecond)
              .withVelocityY(reefAutoAlignSpeeds.vyMetersPerSecond)
              .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
        }
      }
      case AUTO -> {
        drivetrain.setControl(
          driveRobotRelative
            .withVelocityX(autoSpeeds.vxMetersPerSecond)
            .withVelocityY(autoSpeeds.vyMetersPerSecond)
            .withRotationalRate(autoSpeeds.omegaRadiansPerSecond)
            .withDriveRequestType(DriveRequestType.Velocity)
        );
      }
    }
  }
  
  public void setState(DrivetrainState newState) {
    setStateFromRequest(newState);
  }

    private static DrivetrainSubsystem instance;

    public static DrivetrainSubsystem getInstance() {
      if (instance == null) instance = new DrivetrainSubsystem(); // Make sure there is an instance (this will only run once)
      return instance;
  }
}
