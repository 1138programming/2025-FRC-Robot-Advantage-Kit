// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DeviceConstants.*;
import static frc.robot.Constants.HangConstants.*;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*In entire lift:
2 limit switches,
- bottom one autonomus direct
- top one, also there
1 cancoder at bottom
1 Talon to rotate string
*/

public class Hang extends SubsystemBase {
  /** Creates a new Hang. */
  private TalonFX tiltMotor;

  private DutyCycleOut dutyCycleController;

  private Servo lock;

  private DutyCycleEncoder HangEncoder;

  public Hang() {
    tiltMotor = new TalonFX(KHangMotorId);
    HangEncoder =
        new DutyCycleEncoder(
            KHangThroughEncoderId,
            KHangThroughEncoderFullRotationValue,
            KHangThroughEncoderZeroPosition);
    dutyCycleController = new DutyCycleOut(0).withEnableFOC(true);

    lock = new Servo(KHangLock);
  }

  public void tiltHang() {}

  public void tiltHangToPos() {}

  public void setlock(double pos) {
    lock.set(pos);
  }

  public void setHangSpeed(double output) {
    tiltMotor.setControl(dutyCycleController.withOutput(output));
  }

  public void moveHang(double hangSpeed, double pos) {
    tiltMotor.setControl(dutyCycleController.withOutput(hangSpeed));
    lock.set(pos);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("servo", lock.get());
    // This method will be called once per scheduler run
  }
}
