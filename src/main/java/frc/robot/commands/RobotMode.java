package frc.robot.commands;

public class RobotMode {
    public GameMode currentGameMode = GameMode.CORAL;

    public enum GameMode {CORAL, ALGAE}

    public void setCurrentGameMode(GameMode gameMode) {currentGameMode = gameMode;}
    public boolean inAlgaeMode() {return currentGameMode == GameMode.ALGAE;}
    public boolean inCoralMode() {return currentGameMode == GameMode.CORAL;}

    public CycleMode currentCycleMode = CycleMode.REGULAR_CYCLE;
    
    public enum CycleMode {SUPERCYCLE, REGULAR_CYCLE}
    
    public void setCurrentCycleMode(CycleMode cycleMode) {currentCycleMode = cycleMode;}
    public boolean inSupercycleMode() {return currentCycleMode == CycleMode.SUPERCYCLE;}
    public boolean inRegularCycleMode() {return currentCycleMode == CycleMode.REGULAR_CYCLE;}
    
    private static RobotMode instance;

  public static RobotMode getInstance() {
      if (instance == null) instance = new RobotMode(); // Make sure there is an instance (this will only run once)
      return instance;
  }
}

