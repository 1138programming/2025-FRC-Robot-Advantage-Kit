package frc.robot.util;

import frc.robot.subsystems.Arm;

public class SubsystemUtil {
  private Arm arm;

  private static double swerveSpeedFactor = 0.6;

  public SubsystemUtil(Arm arm) {
    this.arm = arm;
  }

  public double getArmSpeed() {
    return arm.getArmSpeed();
  }

  public void setSwerveMaxSpeed(double speedMod) {
    swerveSpeedFactor = speedMod;
  }

  public double getSwerveMaxSpeed() {
    return swerveSpeedFactor;
  }
}
