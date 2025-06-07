package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FmsSubsystem extends SubsystemBase {
  public FmsSubsystem() {
  }

  public static boolean isRedAlliance() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);

    return alliance == Alliance.Red;
  }

  @Override
  public void periodic() {
    //DogLog.log("Fms/Alliance", isRedAlliance() ? "Red" : "Blue");
  }
}