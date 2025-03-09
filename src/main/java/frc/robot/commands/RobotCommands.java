package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.commands.RobotMode.GameMode;
import frc.robot.subsystems.climber.ClimberState;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainState;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.elevator.ElevatorPositions;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.util.List;

import dev.doglog.DogLog;

public class RobotCommands {
  private final RobotManager robot;
  private final Subsystem[] requirements;
  

  public RobotCommands() {
    this.robot = RobotManager.getInstance();
    var requirementsList = List.of(robot.elevator, robot.climber, robot.wrist, robot.elbow, robot.manipulator, robot.kicker);
    requirements = requirementsList.toArray(Subsystem[]::new);
  }

  public Command scoreCommand() {
    return Commands.runOnce(robot::scoreRequest, requirements)
        .andThen(robot.waitForState(RobotState.PREPARE_INVERTED_FROM_IDLE));
  }

  public Command L4ScoreCommand() {
    return Commands.runOnce(robot::scoreRequest, requirements)
        .andThen(robot.waitForState(RobotState.PREPARE_INVERTED_FROM_IDLE));
  }

  public Command retractClimbCommand(){
    return Commands.runOnce(()-> ClimberSubsystem.getInstance().setState(ClimberState.DEEP_CLIMB_RETRACT), requirements)
    .andThen(robot.waitForState(RobotState.DEEP_CLIMB_WAIT));
  }

  public Command L1Command() {
    if (!robot.getState().inverted) {
      return invertIdleCommand() // go to inverted idle
        .andThen(Commands.runOnce(robot::prepareL1Request, requirements)) // Prepare CS (non-inverted)
        .andThen(robot.waitForState(RobotState.WAIT_L1)); // Goes back to idle when we're done intaking
    }else {
      return Commands.runOnce(robot::prepareL1Request, requirements)
        .andThen(robot.waitForState(RobotState.WAIT_L1));
    }
  }
  
  public Command L2Command() {
    if (robot.getState().inverted) {
      return alternateIdleCommand() // go to non-inverted idle
        .andThen(Commands.runOnce(robot::prepareL2Request, requirements)) // Prepare CS (non-inverted)
        .andThen(robot.waitForState(RobotState.WAIT_L2)); // Goes back to idle when we're done intaking
    } else {
      return Commands.runOnce(robot::prepareL2Request, requirements)
          .andThen(robot.waitForState(RobotState.WAIT_L2));
    }
  }

  public Command L3Command() {
    if (robot.getState().inverted) {
      return alternateIdleCommand() // go to non-inverted idle
        .andThen(Commands.runOnce(robot::prepareL3Request, requirements)) // Prepare CS (non-inverted)
        .andThen(robot.waitForState(RobotState.WAIT_L3)); // Goes back to idle when we're done intaking
    } else {
      return Commands.runOnce(robot::prepareL3Request, requirements)
        .andThen(robot.waitForState(RobotState.WAIT_L3));
    }
  }
  public Command L4Command() {
    if (robot.getState().inverted) {
      return alternateIdleCommand() // go to non-inverted idle
      .andThen(Commands.runOnce(robot::prepareL4Request, requirements)); // Prepare CS (non-inverted)
      // .andThen(robot.waitForState(RobotManager.getInstance().isHeightCapped == true ? RobotState.CAPPED_L4 : RobotState.WAIT_L4)); // Goes back to idle when we're done intaking
    } else {
      return Commands.runOnce(robot::prepareL4Request, requirements);
      // .andThen(robot.waitForState(RobotManager.getInstance().isHeightCapped == true ? RobotState.CAPPED_L4 : RobotState.WAIT_L4));
    }
  }
  public Command lowAlgaeCommand() {
    if (robot.getState().inverted) {
      return alternateIdleCommand() // go to non-inverted idle
      .andThen(Commands.runOnce(robot::prepareAlgaeLowRequest, requirements)); // Prepare CS (non-inverted)
      // .andThen(robot.waitForState(RobotManager.getInstance().isHeightCapped == true ? RobotState.CAPPED_L4 : RobotState.WAIT_L4)); // Goes back to idle when we're done intaking
    } else {
      return Commands.runOnce(robot::prepareAlgaeLowRequest, requirements);
      // .andThen(robot.waitForState(RobotManager.getInstance().isHeightCapped == true ? RobotState.CAPPED_L4 : RobotState.WAIT_L4));
    }
  }
  public Command highAlgaeCommand() {
    if (robot.getState().inverted) {
      return alternateIdleCommand() // go to non-inverted idle
      .andThen(Commands.runOnce(robot::prepareAlgaeHighRequest, requirements)); // Prepare CS (non-inverted)
      // .andThen(robot.waitForState(RobotManager.getInstance().isHeightCapped == true ? RobotState.CAPPED_L4 : RobotState.WAIT_L4)); // Goes back to idle when we're done intaking
    } else {
      return Commands.runOnce(robot::prepareAlgaeHighRequest, requirements);
      // .andThen(robot.waitForState(RobotManager.getInstance().isHeightCapped == true ? RobotState.CAPPED_L4 : RobotState.WAIT_L4));
    }
  }

  public Command LowReefCommand() {
    DogLog.log("coral mode", RobotManager.getInstance().currentGameMode == GameMode.CORAL);
    return new ConditionalCommand(L3Command(), lowAlgaeCommand(), () -> RobotManager.getInstance().currentGameMode == GameMode.CORAL);
  }
  
  public Command HighReefCommand() {
    return new ConditionalCommand(L4Command(), highAlgaeCommand(), () -> RobotManager.getInstance().currentGameMode == GameMode.CORAL);
  }

  // public Command algaeHighCommand() {
  //   if (robot.getState().inverted) {
  //     return idleCommand() // go to non-inverted idle
  //     .andThen(Commands.runOnce(robot::prepareAlgaeHighRequest, requirements)); // Prepare CS (non-inverted)
  //     // .andThen(robot.waitForState(RobotManager.getInstance().isHeightCapped == true ? RobotState.CAPPED_L4 : RobotState.WAIT_L4)); // Goes back to idle when we're done intaking
  //   } else {
  //     return Commands.runOnce(robot::prepareAlgaeHighRequest, requirements);
  //     // .andThen(robot.waitForState(RobotManager.getInstance().isHeightCapped == true ? RobotState.CAPPED_L4 : RobotState.WAIT_L4));
  //   }
  // }

  // public Command algaeLowCommand() {
  //   if (robot.getState().inverted) {
  //     return idleCommand() // go to non-inverted idle
  //     .andThen(Commands.runOnce(robot::prepareAlgaeLowRequest, requirements)); // Prepare CS (non-inverted)
  //     // .andThen(robot.waitForState(RobotManager.getInstance().isHeightCapped == true ? RobotState.CAPPED_L4 : RobotState.WAIT_L4)); // Goes back to idle when we're done intaking
  //   } else {
  //     return Commands.runOnce(robot::prepareAlgaeLowRequest, requirements);
  //     // .andThen(robot.waitForState(RobotManager.getInstance().isHeightCapped == true ? RobotState.CAPPED_L4 : RobotState.WAIT_L4));
  //   }
  // }

  public Command alternateIdleCommand() {
    return Commands.runOnce(robot::prepareIdleRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE));
  }

  public Command invertIdleCommand() {
      return Commands.runOnce(robot::prepareInvertedIdleRequest, requirements)
        .andThen(robot.waitForState(RobotState.INVERTED_IDLE));
  }

  public Command idleCommand() {
    return new ConditionalCommand(invertIdleCommand(), stopIntakeAlgaeCommand(), () -> RobotManager.getInstance().currentGameMode == GameMode.CORAL);
  }

  public Command climbCommand() {
    // if (robot.getState().inverted) {
      return alternateIdleCommand()
      .andThen(Commands.runOnce(robot::climbRequest, requirements))
      .andThen(robot.waitForState(RobotState.DEEP_CLIMB_WAIT));
    // } else{
    // return Commands.runOnce(robot::climbRequest, requirements)
    //     .andThen(robot.waitForState(RobotState.DEEP_CLIMB_WAIT));    
    // }
  }

  public Command climbUnwindCommand() {
    if (robot.climber.getState() == ClimberState.DEEP_CLIMB_WAIT) {
      return Commands.runOnce(robot::climbUnwindRequest, requirements)
      .andThen(robot.waitForState(RobotState.DEEP_CLIMB_UNWIND));
    } else {
      return climbCommand();
    }
  }

  public Command climbRetractCommand() {
    if (robot.climber.getState() == ClimberState.DEEP_CLIMB_WAIT) {
      return Commands.runOnce(robot::climbRetractRequest, requirements)
      .andThen(robot.waitForState(RobotState.DEEP_CLIMB_RETRACT));
    } else {
      return climbCommand();
    }
  }

  public Command removeHeightCapCommand() {
    return Commands.runOnce(robot::removeHeightCapRequest);
  }

  public Command applyHeightCapCommand() {
    return Commands.runOnce(robot::applyHeightCapRequest);
  }

  public Command alternateIntakeCommand() {
    if (robot.getState().inverted) {
      return alternateIdleCommand() // go to non-inverted idle
        .andThen(Commands.runOnce(robot::prepareCoralStationRequest, requirements)) // Prepare CS (non-inverted)
        .andThen(robot.waitForState(RobotState.IDLE)); // Goes back to idle when we're done intaking
    }else {
      return Commands.runOnce(robot::prepareCoralStationRequest, requirements)
          .andThen(robot.waitForState(RobotState.IDLE));
    }
  }

  public Command invertedIntakeCommand() {
      if (!robot.getState().inverted) {
        return invertIdleCommand() // go to non-inverted idle
          .andThen(Commands.runOnce(robot::prepareInvertedCoralStationRequest, requirements)) // Prepare CS (inverted)
          .andThen(robot.waitForState(RobotState.INVERTED_IDLE)); // Goes back to inverted idle when we're done intaking
      } else {
        return Commands.runOnce(robot::prepareInvertedCoralStationRequest, requirements)
            .andThen(robot.waitForState(RobotState.INVERTED_IDLE));
      }
  }
  public Command intakeAlgaeCommand() {
    return runOnce(robot::intakeAlgaeRequest, requirements);
  }

  public Command stopIntakeAlgaeCommand() {
    return runOnce(robot::stopIntakeAlgaeRequest, requirements);
  }

  public Command intakeCommand() {
    return new ConditionalCommand(invertedIntakeCommand(), intakeAlgaeCommand(), () -> RobotManager.getInstance().currentGameMode == GameMode.CORAL);
    // if (RobotManager.getInstance().currentGameMode == GameMode.CORAL) {
    //   return invertedIntakeCommand();
    // } else {
    //   return intakeAlgaeCommand();
    // }
  }



  public Command autoCoralStationAlign(){
    return Commands.runOnce(robot::autoCoralStationAlignRequest, CommandSwerveDrivetrain.getInstance())
    .andThen(Commands.waitUntil(()-> DrivetrainSubsystem.getInstance().getState() == DrivetrainState.AUTO || DrivetrainSubsystem.getInstance().getState() == DrivetrainState.TELEOP));
  }

  public Command autoReefAlign(){
    return Commands.runOnce(robot::autoReefAlignRequest, CommandSwerveDrivetrain.getInstance())
    .andThen(Commands.waitUntil(()-> DrivetrainSubsystem.getInstance().getState() == DrivetrainState.AUTO || DrivetrainSubsystem.getInstance().getState() == DrivetrainState.TELEOP));
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
    return Commands.runOnce(robot::algaeModeRequest); 
  }

  public Command coralModeCommand(){
    if(robot.getState().inverted){
      return alternateIdleCommand()
      .andThen(Commands.runOnce(robot::coralModeRequest));
    }
    return Commands.runOnce(robot::coralModeRequest); 
  }

  // public Command climberRetract(){
  //   if()
  // }
}
