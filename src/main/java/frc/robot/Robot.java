package frc.robot;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.RobotCommands;
import frc.robot.commands.RobotManager;
import frc.robot.commands.RobotState;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.vision.LimelightLocalization;
import frc.robot.vision.LimelightState;
import frc.robot.vision.LimelightSubsystem;

import java.lang.reflect.Field;
import java.util.Optional;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import dev.doglog.DogLog;


public class Robot extends TimedRobot{
    public static final double DEFAULT_PERIOD = 0.02;

    public static RobotManager robotManager = RobotManager.getInstance();
    public static RobotCommands robotCommands = new RobotCommands();

    public static Optional<Alliance> alliance = Optional.empty();
    public static final Controls controls = new Controls();

    private SendableChooser<Command> autoChooser;

    public Robot() {
    }

    @Override
    public void robotInit() {
        controls.configureDriverCommands();
        controls.configureOperatorCommands();

        WristSubsystem.getInstance().wristMotor.setPosition(0.38);
        LimelightSubsystem.getInstance().setState(LimelightState.DISABLED);

        NamedCommands.registerCommand("idle", Robot.robotCommands.alternateIdleCommand());
        NamedCommands.registerCommand("inverted idle", Robot.robotCommands.invertIdleCommand());
        NamedCommands.registerCommand("score", Robot.robotCommands.scoreCommand());
        NamedCommands.registerCommand("L1", Robot.robotCommands.L1Command());
        NamedCommands.registerCommand("L2", Robot.robotCommands.L2Command());
        NamedCommands.registerCommand("L3", Robot.robotCommands.L3Command());
        NamedCommands.registerCommand("L4", Robot.robotCommands.L4Command());
        NamedCommands.registerCommand("wait for inverted idle", robotManager.waitForState(RobotState.INVERTED_IDLE));
        NamedCommands.registerCommand("wait for prepare inverted idle", robotManager.waitForState(RobotState.PREPARE_INVERTED_IDLE));
        NamedCommands.registerCommand("wait for post intake", robotManager.waitForState(RobotState.POST_INVERTED_CORAL_STATION_INTAKE));
        NamedCommands.registerCommand("wait for L4", robotManager.waitForState(RobotState.WAIT_L4));
        NamedCommands.registerCommand("limelight state to auto reef", Commands.runOnce(() -> LimelightSubsystem.getInstance().setStateFromRequest(LimelightState.AUTO_REEF)));
        NamedCommands.registerCommand("limelight state to auto coral station", Commands.runOnce(() -> LimelightSubsystem.getInstance().setStateFromRequest(LimelightState.AUTO_CORAL_STATION)));
        NamedCommands.registerCommand("remove height cap", Robot.robotCommands.removeHeightCapCommand());
        NamedCommands.registerCommand("auto coral station align", Robot.robotCommands.autoCoralStationAlign());
        NamedCommands.registerCommand("auto reef align", Robot.robotCommands.autoReefAlign());
        NamedCommands.registerCommand("apply height cap", Robot.robotCommands.applyHeightCapCommand());
        NamedCommands.registerCommand("climb", Robot.robotCommands.climbCommand());
        NamedCommands.registerCommand("intake", Robot.robotCommands.intakeCommand());
        NamedCommands.registerCommand("inverted intake", Robot.robotCommands.invertedIntakeCommand());
        NamedCommands.registerCommand("home", Robot.robotCommands.homeCommand());
        NamedCommands.registerCommand("apply height cap", Robot.robotCommands.applyHeightCapCommand());
        NamedCommands.registerCommand("set drivetrain auto", Robot.robotCommands.setDrivetrainAuto());

        autoChooser = AutoBuilder.buildAutoChooser();
        LED led = new LED(robotManager);
        FieldConstants.getInstance().logBranches();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putData(autoChooser);
        if (alliance.isEmpty()) {
            alliance = DriverStation.getAlliance();
        }
    }

    @Override
    public void disabledPeriodic() {
        LimelightSubsystem.getInstance().setState(LimelightState.DISABLED);
        alliance = DriverStation.getAlliance();
    }


    @Override
    public void teleopInit() {
        LimelightSubsystem.getInstance().setState(LimelightState.DRIVE);
        CommandScheduler.getInstance().schedule(Robot.robotCommands.applyHeightCapCommand()
            .andThen(Robot.robotCommands.setDrivetrainTeleop())
            .andThen(Robot.robotCommands.invertIdleCommand()));
    }

    @Override
    public void teleopPeriodic() {
        
      }


    @Override
    public void teleopExit() {
        controls.driver.rumble(0);
    }

    @Override
    public void disabledInit() {
    }
    
    @Override
    public void autonomousInit() {
        
        if (autoChooser.getSelected() != null)
            autoChooser.getSelected().schedule();
            DogLog.log("Selected Auto", autoChooser.getSelected().getName());
    }

    @Override
  public void autonomousPeriodic() {}


    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
        }
    }