package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.commands.RobotMode.AlgaeScoreMode;
import frc.robot.commands.RobotMode.CycleMode;
import frc.robot.commands.RobotMode.GameMode;
import frc.robot.commands.RobotMode.IntakeMode;
import frc.robot.commands.RobotMode.L1Row;
import frc.robot.subsystems.climber.ClimberState;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainState;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.elbow.ElbowSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.vision.LimelightLocalization;

import static edu.wpi.first.wpilibj2.command.Commands.none;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.util.List;

import dev.doglog.DogLog;

public class RobotCommands {
  private final RobotManager robot;
  private final Subsystem[] requirements;
  

  public RobotCommands() {
    this.robot = RobotManager.getInstance();
    var requirementsList = List.of(robot.elevator, robot.climber, robot.wrist, robot.elbow, robot.manipulator);
    requirements = requirementsList.toArray(Subsystem[]::new);
  }
  
  public Command scoreCommand() {
    return Commands.runOnce(robot::scoreRequest, requirements)
        .andThen(Commands.waitUntil(() -> 
        robot.getState() == RobotState.PREPARE_INVERTED_IDLE || robot.getState() == RobotState.PREPARE_IDLE || robot.getState() == RobotState.PRE_SUPERCYCLE_HIGH_ALGAE || robot.getState() == RobotState.PRE_SUPERCYCLE_LOW_ALGAE));
  }

  public Command algaeScoreCommand() {
    return Commands.runOnce(robot::algaeScoreRequest, requirements)
        .andThen(Commands.waitUntil(() -> 
        robot.getState() == RobotState.PREPARE_IDLE || robot.getState() == RobotState.PREPARE_REMOVE_ALGAE_HIGH));
  }

  public Command intakeIdleCommand() {
    return new ConditionalCommand(invertIdleCommand(), algaeIntakeIdleCommand(), () -> RobotManager.getInstance().currentGameMode == GameMode.CORAL);
  }

  public Command algaeIntakeIdleCommand() {
    return new ConditionalCommand(algaeIntakeRegularIdleCommand(), algaeIntakeFailsafeIdleCommand(), () -> RobotManager.getInstance().currentIntakeMode == IntakeMode.NORMAL);
  }

  public Command algaeIntakeRegularIdleCommand() {
    return new ConditionalCommand((none()), algaeIdleCommand(), () -> (RobotManager.getInstance().getState() == RobotState.PREPARE_POST_GROUND_ALGAE_INTAKE) || (RobotManager.getInstance().getState() == RobotState.POST_GROUND_ALGAE_INTAKE) || (RobotManager.getInstance().getState() == RobotState.GROUND_ALGAE_INTAKE));
  }

  public Command algaeIntakeFailsafeIdleCommand() {
    return new ConditionalCommand((none()), algaeIdleCommand(), () -> (RobotManager.getInstance().getState() == RobotState.FAILSAFE_PREPARE_POST_GROUND_ALGAE_INTAKE) || (RobotManager.getInstance().getState() == RobotState.FAILSAFE_POST_GROUND_ALGAE_INTAKE) || (RobotManager.getInstance().getState() == RobotState.FAILSAFE_GROUND_ALGAE_INTAKE));
  }

  public Command L4ScoreCommand() {
    return Commands.runOnce(robot::scoreRequest, requirements)
        .andThen(robot.waitForState(RobotState.PREPARE_INVERTED_FROM_IDLE));
  }

  public Command failsafeCommand() {
    return Commands.runOnce(robot::failsafeOverrideRequest, requirements);
  }

  public Command retractClimbCommand(){
    return Commands.runOnce(()-> ClimberSubsystem.getInstance().setState(ClimberState.DEEP_CLIMB_RETRACT), requirements)
    .andThen(robot.waitForState(RobotState.DEEP_CLIMB_WAIT));
  }
  public Command L2MultiCommand() {
    return new ConditionalCommand(L2Command(), algaeIdleCommand(), () -> RobotManager.getInstance().currentGameMode == GameMode.CORAL);
  }
  public Command L1MultiCommand() {
    return new ConditionalCommand(L1Command(), algaeIdleCommand(), () -> RobotManager.getInstance().currentGameMode == GameMode.CORAL);
  }
  public Command L1Command() {
    return new ConditionalCommand(invertIdleCommand().andThen(Commands.runOnce(robot::prepareL1Request, requirements)).andThen(robot.waitForState(RobotState.WAIT_L1)), 
    Commands.runOnce(robot::prepareL1Request, requirements).andThen(robot.waitForState(RobotState.WAIT_L1)), 
    () -> !robot.getState().inverted);
  }

  public Command L1ToggleCommand() {
    return new ConditionalCommand(Commands.runOnce(() -> RobotMode.getInstance().setCurrentL1Mode(L1Row.HIGHTROUGH)), Commands.runOnce(() -> RobotMode.getInstance().setCurrentL1Mode(L1Row.LOWTROUGH)), () -> RobotMode.getInstance().inLowL1Mode())
    .andThen(Commands.runOnce(() -> ElevatorSubsystem.getInstance().setL1Row())
    .andThen(Commands.runOnce(() -> ElbowSubsystem.getInstance().setL1Row()))
    .andThen(Commands.runOnce(() -> WristSubsystem.getInstance().setL1Row())));
  }

  public Command IntakeDisableCommand() {
    return Commands.runOnce(robot::intakeFailsafeRequest);
  }

  public Command IntakeEnableCommand() {
    return Commands.runOnce(robot::regularIntakeRequest);
  }

  public Command L2Command() {
    return new ConditionalCommand(algaeIdleCommand().andThen(Commands.runOnce(robot::prepareL2Request, requirements)).andThen(robot.waitForState(RobotState.WAIT_L2)), 
    Commands.runOnce(robot::prepareL2Request, requirements).andThen(robot.waitForState(RobotState.WAIT_L2)), 
    () -> robot.getState().inverted);
  }

  public Command L3Command() {
    return new ConditionalCommand(algaeIdleCommand().andThen(Commands.runOnce(robot::prepareL3Request, requirements)).andThen(robot.waitForState(RobotState.WAIT_L3)), 
    Commands.runOnce(robot::prepareL3Request, requirements).andThen(robot.waitForState(RobotState.WAIT_L3)), 
    () -> robot.getState().inverted);
  }
  public Command L4Command() {
    return new ConditionalCommand(algaeIdleCommand().andThen(Commands.runOnce(robot::prepareL4Request, requirements)), 
    Commands.runOnce(robot::prepareL4Request, requirements), 
    () -> robot.getState().inverted);
  }
  public Command lowAlgaeCommand() {
    return new ConditionalCommand(algaeIdleCommand().andThen(Commands.runOnce(robot::prepareAlgaeLowRequest, requirements)),
    Commands.runOnce(robot::prepareAlgaeLowRequest, requirements), 
    () -> robot.getState().inverted);
  }
  public Command highAlgaeCommand() {
    return new ConditionalCommand(algaeIdleCommand().andThen(Commands.runOnce(robot::prepareAlgaeHighRequest, requirements)),
    Commands.runOnce(robot::prepareAlgaeHighRequest, requirements), 
    () -> robot.getState().inverted);
  }

  public Command setProcessorCommand() {
    return Commands.runOnce(robot::setProcessorRequest, requirements);
  }

  public Command ProcessorCommand() {
    return new ConditionalCommand(L1Command(), setProcessorCommand(), () -> RobotManager.getInstance().currentGameMode == GameMode.CORAL);
  }
  
  public Command LowReefCommand() {
    DogLog.log("coral mode", RobotManager.getInstance().currentGameMode == GameMode.CORAL);
    return new ConditionalCommand(L3Command(), lowAlgaeCommand(), () -> RobotManager.getInstance().currentGameMode == GameMode.CORAL);
  }
  
  public Command HighReefCommand() {
    return new ConditionalCommand(L4Command(), highAlgaeCommand(), () -> RobotManager.getInstance().currentGameMode == GameMode.CORAL);
  }

  public Command algaeIdleCommand() {
    return Commands.runOnce(robot::prepareIdleRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE));
  }

  public Command invertIdleCommand() {
      return Commands.runOnce(robot::prepareInvertedIdleRequest, requirements)
        .andThen(robot.waitForState(RobotState.INVERTED_IDLE));
  }

  public Command scoreIdleCommand() {
    return new ConditionalCommand(supercycleCommand(), idleCommand(), () -> (RobotManager.getInstance().currentCycleMode == CycleMode.SUPERCYCLE) && (RobotManager.getInstance().currentGameMode == GameMode.CORAL) && !(RobotManager.getInstance().getState() == RobotState.SCORE_L1));
  }

  public Command idleCommand() {
    return new ConditionalCommand(invertIdleCommand(), algaeIdleMultiCommand(), () -> RobotManager.getInstance().currentGameMode == GameMode.CORAL);
  }

  public Command algaeIdleMultiCommand() {
    return new ConditionalCommand(highAlgaeCommand(), algaeIdleCommand(), () -> RobotManager.getInstance().getState() == RobotState.SCORE_ALGAE);
  }

  public Command supercycleCommand() {
    return Commands.runOnce(robot::algaeModeRequest, requirements)
      .andThen(Robot.robotCommands.supercycleAlgaeCommand())
      .andThen(Commands.waitUntil(() -> 
        robot.getState() == RobotState.PRE_SUPERCYCLE_HIGH_ALGAE || robot.getState() == RobotState.PRE_SUPERCYCLE_LOW_ALGAE));
  }

  public Command supercycleAlgaeCommand() {
    return new ConditionalCommand(highAlgaeCommand(), lowAlgaeCommand(), () -> FieldConstants.getInstance().isNearHighAlgae());
  }

  public Command climbCommand() {
      return algaeIdleCommand()
      .andThen(Commands.runOnce(robot::climbRequest, requirements))
      .andThen(robot.waitForState(RobotState.DEEP_CLIMB_WAIT));
  }

  public Command failsafeClimbCommand() {
    return Commands.runOnce(robot::failsafeClimbRequest, requirements)
    .andThen(robot.waitForState(RobotState.FAILSAFE_DEEP_CLIMB_WAIT));
}

  public Command climbUnwindCommand() {
    return new ConditionalCommand(Commands.runOnce(robot::climbUnwindRequest), climbCommand(), () -> (RobotManager.getInstance().getState() == RobotState.DEEP_CLIMB_WAIT) || (RobotManager.getInstance().getState() == RobotState.DEEP_CLIMB_RETRACT));
  }

  public Command climbIdleCommand() {
    return new ConditionalCommand(Commands.runOnce(robot::climbIdleRequest), climbCommand(), () -> RobotManager.getInstance().getState() == RobotState.DEEP_CLIMB_WAIT || RobotManager.getInstance().getState() == RobotState.DEEP_CLIMB_RETRACT || RobotManager.getInstance().getState() == RobotState.DEEP_CLIMB_UNWIND);
  }

  public Command climbRetractCommand() {
    return new ConditionalCommand(Commands.runOnce(robot::climbRetractRequest), 
    climbCommand(), 
    () -> (RobotManager.getInstance().getState() == RobotState.DEEP_CLIMB_WAIT) || (RobotManager.getInstance().getState() == RobotState.DEEP_CLIMB_RETRACT));
  }

  public Command failsafeClimbRetractCommand() {
    return Commands.runOnce(robot::failsafeClimbRetractRequest, requirements)
    .andThen(robot.waitForState(RobotState.FAILSAFE_DEEP_CLIMB_WAIT));
  }


  public Command removeHeightCapCommand() {
    return Commands.runOnce(robot::removeHeightCapRequest);
  }

  public Command applyHeightCapCommand() {
    return Commands.runOnce(robot::applyHeightCapRequest);
  }

  public Command algaeScoreSwapCommand() {
    return new ConditionalCommand(Commands.runOnce(() -> RobotMode.getInstance().setCurrentAlgaeScoreMode(AlgaeScoreMode.REGULAR)), Commands.runOnce(() -> RobotMode.getInstance().setCurrentAlgaeScoreMode(AlgaeScoreMode.FRONT)), () -> RobotMode.getInstance().inFrontAlgaeScoreMode())
    .andThen(new ConditionalCommand(Commands.runOnce(robot::frontAlgaeScoreRequest), Commands.runOnce(robot::normalAlgaeScoreRequest), () -> RobotMode.getInstance().inFrontAlgaeScoreMode()));
  }

  public Command alternateIntakeCommand() {
    return new ConditionalCommand(
      algaeIdleCommand()
      .andThen(Commands.runOnce(robot::prepareCoralStationRequest, requirements))
      .andThen(robot.waitForState(RobotState.IDLE)), 
        
      Commands.runOnce(robot::prepareCoralStationRequest, requirements)
      .andThen(robot.waitForState(RobotState.IDLE)),
      
      () -> (robot.getState().inverted));
  }

  public Command invertedIntakeCommand() {
    return new ConditionalCommand(
      invertIdleCommand() // go to non-inverted idle
      .andThen(Commands.runOnce(robot::prepareInvertedCoralStationRequest, requirements)) // Prepare CS (inverted)
      .andThen(robot.waitForState(RobotState.INVERTED_IDLE)), // Goes back to inverted idle when we're done intaking
        
      Commands.runOnce(robot::prepareInvertedCoralStationRequest, requirements)
      .andThen(robot.waitForState(RobotState.INVERTED_IDLE)),
      
      () -> (!robot.getState().inverted));
  }

  // public Command funnelIntakeCommand() {
  //   return new ConditionalCommand(
  //     invertIdleCommand() // go to non-inverted idle
  //     .andThen(Commands.runOnce(robot::prepareFunnelRequest, requirements)) // Prepare CS (inverted)
  //     .andThen(robot.waitForState(RobotState.INVERTED_IDLE)), // Goes back to inverted idle when we're done intaking
        
  //     Commands.runOnce(robot::prepareInvertedCoralStationRequest, requirements)
  //     .andThen(robot.waitForState(RobotState.INVERTED_IDLE)),
      
  //     () -> (!robot.getState().inverted));
  // }

  public Command intakeAlgaeCommand() {
    return runOnce(robot::intakeAlgaeRequest, requirements);
  }

  public Command stopIntakeAlgaeCommand() {
    return runOnce(robot::stopIntakeAlgaeRequest, requirements);
  }

  public Command intakeGroundAlgaeCommand() {
    return algaeIdleCommand().andThen(runOnce(robot::intakeGroundAlgaeRequest, requirements));
  }



  public Command intakeCommand() {
    return new ConditionalCommand(invertedIntakeCommand(), intakeGroundAlgaeCommand(), () -> RobotManager.getInstance().currentGameMode == GameMode.CORAL);
  }

  public Command autoCoralStationAlign(){
    return Commands.runOnce(robot::autoCoralStationAlignRequest, CommandSwerveDrivetrain.getInstance())
    .andThen(Commands.waitUntil(()-> DrivetrainSubsystem.getInstance().getState() == DrivetrainState.AUTO || DrivetrainSubsystem.getInstance().getState() == DrivetrainState.TELEOP));
  }

  public Command autoAlignCommand(){
    return new ConditionalCommand(autoReefAlign(), autoAlgaeAlign(), () -> RobotManager.getInstance().currentGameMode == GameMode.CORAL);
  }

  public Command autoAlgaeAlign(){
    return new ConditionalCommand(Commands.runOnce(robot::autoAlgaeAlignRequest, CommandSwerveDrivetrain.getInstance())
    .andThen(Commands.waitUntil((
    )-> DrivetrainSubsystem.getInstance().getState() == DrivetrainState.AUTO || DrivetrainSubsystem.getInstance().getState() == DrivetrainState.TELEOP))
    ,none()
    ,() ->  DriverStation.isAutonomous() || CommandSwerveDrivetrain.getInstance().isNear(LimelightLocalization.getInstance().getAdjustedAlgaePose(), 0.75));
  }

  public Command setDrivetrainAuto(){
    return Commands.runOnce(()-> DrivetrainSubsystem.getInstance().setState(DrivetrainState.AUTO));
  }

  public Command setDrivetrainTeleop(){
    return Commands.runOnce(()-> DrivetrainSubsystem.getInstance().setState(DrivetrainState.TELEOP));
  }

  public Command homeCommand(){
    return Commands.runOnce(robot::homeRequest, requirements)
      .andThen(robot.waitForState(RobotState.PREPARE_HOMING));    
  }

  public Command algaeModeCommand(){
    return algaeIdleCommand().andThen(Commands.runOnce(robot::algaeModeRequest));
  }

  public Command coralModeCommand(){ 
    return invertIdleCommand().andThen(Commands.runOnce(robot::coralModeRequest));
  }

  public Command cycleModeCommand(){
    return new ConditionalCommand(supercycleModeCommand(), regularCycleModeCommand(), () -> RobotManager.getInstance().currentCycleMode == CycleMode.REGULAR_CYCLE);
  }
  
  public Command autoReefAlign(){
    return new ConditionalCommand(Commands.runOnce(robot::autoReefAlignRequest, CommandSwerveDrivetrain.getInstance())
    .andThen(Commands.waitUntil(()-> DrivetrainSubsystem.getInstance().getState() == DrivetrainState.AUTO || DrivetrainSubsystem.getInstance().getState() == DrivetrainState.TELEOP))
    .andThen(new ConditionalCommand(scoreCommand(), none(), () -> RobotManager.getInstance().getState() == RobotState.WAIT_L2 || RobotManager.getInstance().getState() == RobotState.WAIT_L3))
    ,none()
    ,() ->  DriverStation.isAutonomous() || CommandSwerveDrivetrain.getInstance().isNear(LimelightLocalization.getInstance().getAdjustedBranchPose(), 0.75));
  }

  public Command supercycleModeCommand(){
    return Commands.runOnce(robot::supercycleRequest);
  }

  public Command regularCycleModeCommand(){
    return Commands.runOnce(robot::regularCycleRequest);
  }
  

  public Command changeClimberPID(){
    return Commands.runOnce(() -> ClimberSubsystem.getInstance().setRetractConfig());
  }
}
