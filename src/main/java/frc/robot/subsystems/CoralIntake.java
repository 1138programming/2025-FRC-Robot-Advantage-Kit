// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DeviceConstants.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
  public DigitalInput CoralIntakeLimitSwitch;
  private SparkFlex CoralIntakeMotor;
  private SparkBaseConfig sparkFlexConfig;

  public CoralIntake() {
    CoralIntakeMotor = new SparkFlex(KCoralIntakeMotorId, MotorType.kBrushless);
    sparkFlexConfig = new SparkFlexConfig();
    sparkFlexConfig.smartCurrentLimit(65);
    CoralIntakeMotor.configure(
        sparkFlexConfig,
        com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);
    CoralIntakeLimitSwitch = new DigitalInput(KCoralIntakeMotorLimitSwitch);
  }

  public boolean getObjectPossesion() {
    return CoralIntakeLimitSwitch.get();
  }

  public void setCoralIntakeSpeed(double speed) {
    CoralIntakeMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
