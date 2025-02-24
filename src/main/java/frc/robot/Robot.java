package frc.robot;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.RobotCommands;
import frc.robot.commands.RobotManager;
import frc.robot.subsystems.LED.LED;

import java.util.Optional;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;


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

        NamedCommands.registerCommand("idle", Robot.robotCommands.idleCommand());
        NamedCommands.registerCommand("inverted idle", Robot.robotCommands.invertIdleCommand());
        NamedCommands.registerCommand("score", Robot.robotCommands.scoreCommand());
        NamedCommands.registerCommand("L1", Robot.robotCommands.L1Command());
        NamedCommands.registerCommand("L2", Robot.robotCommands.L2Command());
        NamedCommands.registerCommand("L3", Robot.robotCommands.L3Command());
        NamedCommands.registerCommand("L4", Robot.robotCommands.L4Command());
        NamedCommands.registerCommand("remove height cap", Robot.robotCommands.removeHeightCapCommand());
        NamedCommands.registerCommand("auto coral station align", Robot.robotCommands.autoCoralStationAlign());
        NamedCommands.registerCommand("auto reef align", Robot.robotCommands.autoReefAlign());
        NamedCommands.registerCommand("apply height cap", Robot.robotCommands.applyHeightCapCommand());
        NamedCommands.registerCommand("climb", Robot.robotCommands.climbCommand());
        NamedCommands.registerCommand("intake", Robot.robotCommands.intakeCommand());
        NamedCommands.registerCommand("inverted intake", Robot.robotCommands.invertedIntakeCommand());
        NamedCommands.registerCommand("home", Robot.robotCommands.homeCommand());
        NamedCommands.registerCommand("remove height cap", Robot.robotCommands.removeHeightCapCommand());
        NamedCommands.registerCommand("set drivetrain auto", Robot.robotCommands.setDrivetrainAuto());

        autoChooser = AutoBuilder.buildAutoChooser();
        LED led = new LED(robotManager);
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
        alliance = DriverStation.getAlliance();
    }


    @Override
    public void teleopInit() {

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