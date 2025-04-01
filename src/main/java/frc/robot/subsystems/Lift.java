package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.DeviceConstants.*;
import static frc.robot.Constants.LiftConstants.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/*In entire lift:
2 limit switches,
- bottom one autonomus direct
- top one, also there
1 cancoder at bottom
1 Talon to rotate string
*/

public class Lift extends SubsystemBase {
  private TalonFX liftMotor;
  private TalonFXConfiguration liftMotorConfig;
  private MotorOutputConfigs motorOutputConfigs;

  private CANcoder liftCANCoder;
  private CANcoderConfiguration canCoderConfig;

  private DigitalInput toplimitSwitch;
  private DigitalInput bottomlimitSwitch;

  private double startingAngle;

  private VoltageOut voltageController;

  private PositionVoltage m_request;
  // in init function, set slot 0 gains

  private PositionVoltage positionVoltageController;

  private TrapezoidProfile m_profile;

  private TrapezoidProfile.State m_goal;
  private TrapezoidProfile.State m_setpoint;

  private boolean manualControl = false;

  private double offset;

  private double liftOffset;

  private final SysIdRoutine m_sysIdRoutineLift =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(5), // Use dynamic voltage of 7 V
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdLift_State", state.toString())),
          new SysIdRoutine.Mechanism(volts -> setLiftElevatorVoltage(volts), null, this));

  public Lift() {
    // Devices
    liftMotor = new TalonFX(KLiftMotorID);
    liftCANCoder = new CANcoder(KLiftCANCoderID);
    toplimitSwitch = new DigitalInput(KLiftTopLimitSwitch);
    bottomlimitSwitch = new DigitalInput(KLiftBottomLImitSwitch);

    // Cancoder Config
    canCoderConfig = new CANcoderConfiguration();

    canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    canCoderConfig.MagnetSensor.MagnetOffset = 0;

    liftOffset = 1.5;

    liftCANCoder.getConfigurator().apply(canCoderConfig);

    // Lift Motor Config
    liftMotorConfig = new TalonFXConfiguration();

    liftMotorConfig.Feedback.FeedbackRemoteSensorID = liftCANCoder.getDeviceID();
    liftMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    liftMotorConfig.Feedback.SensorToMechanismRatio = 1;
    liftMotorConfig.Feedback.RotorToSensorRatio = 46.67;
    liftMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    liftMotor.getConfigurator().apply(liftMotorConfig);

    positionVoltageController = new PositionVoltage(0).withEnableFOC(true);

    m_profile =
        new TrapezoidProfile(new TrapezoidProfile.Constraints(KMaxVoltage, KMaxAcceleration));

    voltageController = new VoltageOut(0);

    // Lift Controller Configuration
    var slot0Configs =
        new Slot0Configs()
            .withKP(6.4238)
            .withKI(0)
            .withKD(0)
            .withKS(0.29752)
            .withKV(4.4829)
            .withKA(1.1514)
            .withKG(0.25628)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    liftMotor.getConfigurator().apply(slot0Configs);

    m_goal = new TrapezoidProfile.State(0, 0);

    m_setpoint = new TrapezoidProfile.State();
    offset = 0;
  }

  public void setLiftElevatorVoltage(Voltage volts) {
    liftMotor.setControl(voltageController.withOutput(volts));
  }

  public void setLiftElevatorSpeed(double speed) {
    liftMotor.set(speed);
  }

  public void liftStop() {
    liftMotor.set(0);
  }

  public void getLiftOffset() {
    if (bottomlimitSwitch.get()) {
      offset = (0 - liftCANCoder.getAbsolutePosition().getValueAsDouble());
    }
    if (toplimitSwitch.get()) {
      offset = (6.7 - liftCANCoder.getAbsolutePosition().getValueAsDouble());
    }
  }

  public void MoveLiftToSetPositionCTRE(double position) {

    // m_goal = new TrapezoidProfile.State(position + offset, 0); // new goal position
    m_goal = new TrapezoidProfile.State(position + offset, 0); // new goal position
    m_setpoint = m_profile.calculate(0.020, m_setpoint, m_goal);

    positionVoltageController.Position = m_setpoint.position;
    positionVoltageController.Velocity = m_setpoint.velocity;
    if (!manualControl) {
      if (!(positionVoltageController.Velocity < 0 && bottomlimitSwitch.get())
          || !(positionVoltageController.Velocity > 0 && toplimitSwitch.get())) {
        liftMotor.setControl(positionVoltageController);
      } else {
        liftMotor.stopMotor();
      }
    }
  }

  public double getLiftCANCoder() {
    return liftCANCoder.getPosition().getValueAsDouble() * 360;
  }

  public void setLiftManualControl() {
    manualControl = !manualControl;
  }

  public Command sysIdQuasistaticLift(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineLift.quasistatic(direction);
  }

  public Command sysIdDynamicLift(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineLift.dynamic(direction);
  }

  public double getSpeedLift() {
    return liftMotor.get();
  }
  // Limit Switches
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Lift Top", toplimitSwitch.get());
    SmartDashboard.putBoolean("Lift Bottom", bottomlimitSwitch.get());
    // SmartDashboard.putNumber("Lift encoder", liftCANCoder.getPosition().getValueAsDouble() +
    // liftOffset);
    SmartDashboard.putNumber("Lift encoder", liftCANCoder.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("Lift Manual Control", manualControl);
    // getLiftOffset();
  }
}
