package frc.robot;

public final class Constants {
  public static final class DrivetrainConstants{
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 5.94;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 300;
  }
public static final class ClimberConstants{
  public static final double P = 320;
  public static final double I = 0;
  public static final double D = 0;
  public static final double G = 0;
  public static final double DeployMotionMagicAcceleration = 100; //100
  public static final double DeployMotionMagicCruiseVelocity = 250; //250
  public static final double RetractMotionMagicAcceleration = 0.44;
  public static final double RetractMotionMagicCruiseVelocity = 0.44;
  public static final double MotionMagicJerk = 200;
}

public static final class ElevatorConstants{
  public static final double encoderOffset = 0;
  public static final double homingStallCurrent = 35;
  public static final double P = 40; //40
  public static final double I = 0;
  public static final double D = 0;
  public static final double G = 0.5075;
  public static final double MotionMagicAcceleration = 50; //40
  public static final double MotionMagicCruiseVelocity = 250; //250
  public static final double MotionMagicJerk = 300; //300
}

public static final class WristConstants{
  public static final double homingStallCurrent = 12;
  public static final double P = 24; //24
  public static final double I = 0;
  public static final double D = 0;
  public static final double G = 0.065;
  public static final double MotionMagicAcceleration = 140;//100
  public static final double MotionMagicCruiseVelocity = 50;
  public static final double MotionMagicJerk = 1000;//95
}

public static final class ElbowConstants{
  public static final double homingStallCurrent = 10;
  public static final double P = 40; //40
  public static final double I = 0;
  public static final double D = 0;
  public static final double G = 0.205;
  public static final double MotionMagicAcceleration = 56; //40
  public static final double MotionMagicCruiseVelocity = 50;
  public static final double MotionMagicJerk = 420; //300
}

public static final class IntakeConstants{
  public static final double P = 120; //40
  public static final double I = 0;
  public static final double D = 0;
  public static final double G = 0;
  public static final double MotionMagicAcceleration = 10; //40
  public static final double MotionMagicCruiseVelocity = 10;
  public static final double MotionMagicJerk = 100; //300
}

public static final class ManipulatorConstants{
  public static final double coralStallCurrent = 20;
  public static final double algaeStallCurrent = 85;
}

public static final class ClimberWheelConstants{
  public static final double cageStallCurrent = 60;
}


public static final class OIConstants {
    public static final int DRIVER_CONTROLLER_ID = 0;
    public static final int OPERATOR_CONTROLLER_ID = 1;
}
}

