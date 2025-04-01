package frc.robot.subsystems;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.DeviceConstants.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * Designer intends for arm to be prefrebaly human controlled and to have specific set points:
 *
 * -High
 * -Mid
 * -Low
 * -Bottom
 * -Store
 * -Intake off ground
 * -Intake from player
 */
public class Arm extends SubsystemBase {

  private TalonFX tiltMotor;

  private double speedModifier = 0.6;

  private TalonFXConfiguration motorConfig;

  private Slot0Configs Slot0Configs;

  private PIDController armPidController;

  private ArmFeedforward armFeedforward;

  private DigitalInput bottomLimitSwitch;

  private VoltageOut voltageController;

  private final PositionVoltage positionVoltageController;

  private final DutyCycleOut positionDutyCycleController;

  private final TrapezoidProfile m_TrapezoidProfile; // used for motion profiling

  private TrapezoidProfile.State m_goal; // used for motion profiling -> the current target position

  private TrapezoidProfile.State
      m_setpoint; // used for motion profiling -> the current position to use for PID
  // control

  private DutyCycleEncoder tiltThroughBoreEncoder;

  private boolean armManualControl;

  // private final SysIdRoutine m_sysIdRoutineArm = new SysIdRoutine(
  //         new SysIdRoutine.Config(
  //                 null, // Use default ramp rate (1 V/s)
  //                 Volts.of(5), // Use dynamic voltage of 7 V
  //                 null, // Use default timeout (10 s)
  //                 // Log state with SignalLogger class
  //                 state -> SignalLogger.writeString("SysIdArm_State", state.toString())),
  //         new SysIdRoutine.Mechanism(
  //                 volts -> setMotorVoltage(volts),
  //                 null,
  //                 this));

  public Arm() {

    tiltMotor = new TalonFX(KTiltArmId);

    tiltThroughBoreEncoder =
        new DutyCycleEncoder(KTiltThroughEncoderId, KTiltThroughEncoderFullRotationValue, 315);
    tiltThroughBoreEncoder.setInverted(false);

    motorConfig = new TalonFXConfiguration();
    // motorConfig.Feedback.SensorToMechanismRatio = 1.0;
    // motorConfig.Feedback.RotorToSensorRatio = 46.67; //
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // WPI Pid
    armPidController = new PIDController(0.02, KArmControlI, 0);

    armFeedforward = new ArmFeedforward(0, 0, 0, 0);

    positionDutyCycleController = new DutyCycleOut(0).withEnableFOC(true);

    // CTRE pid
    Slot0Configs = new Slot0Configs();
    // config.Slot0.kS = ArmConstants.KArmControlS; } for later use if wish to add
    // values to overcome static friction
    // config.Slot0.kV = ArmConstants.KArmControlV; }
    Slot0Configs.kP = KArmControlP;
    Slot0Configs.kI = KArmControlI;
    Slot0Configs.kD = KArmControlD;

    positionVoltageController = new PositionVoltage(0).withEnableFOC(true);
    m_TrapezoidProfile =
        new TrapezoidProfile(new TrapezoidProfile.Constraints(KMaxVoltage, KMaxAcceleration));
    // set first goal to stored position of arm, will change later upon use input

    // accurate position tracking throughout tournaments
    m_goal = new TrapezoidProfile.State(0, 0);
    m_setpoint = new TrapezoidProfile.State();

    voltageController = new VoltageOut(0).withEnableFOC(true);

    tiltMotor
        .getConfigurator()
        .apply(motorConfig); // apply the configuration to the motor, add PID values
    tiltMotor
        .getConfigurator()
        .apply(Slot0Configs); // apply the configuration to the motor, add PID values

    tiltMotor.setNeutralMode(
        NeutralModeValue.Brake); // set the motor to brake mode so arm is precise

    armManualControl = false;

    bottomLimitSwitch = new DigitalInput(KArmLimitSwitch);

    speedModifier = 0.6;
  }

  public void tiltArmManually(double speed) {

    // //check if for top sensor
    // if (hallSensorTop.get() && speed > 0) {
    // speed = 0;
    // }

    // //check if for bottom sensor
    // if (hallSensorBottom.get() && speed < 0) {
    // speed = 0;

    // }

    tiltMotor.set(speed);

    // double currentPosition = getTiltEncoder();
    // m_setpoint = new TrapezoidProfile.State(currentPosition, 0);
  }

  public void armHold() {
    double pos = tiltThroughBoreEncoder.get();
    if (armManualControl) {
      tiltArmToSetPositionWPI(pos);
    }
  }

  // TODO: make sure arm position does not exceed physical limits of arm / limit
  // switch
  // position should be in rotations
  // Set up steamdeck buttons to correspond to values
  public void tiltArmToSetPositionCTRE(double position) {
    // Should call setArmPosition encorperating steamdeck button values to
    // correspond to set points
    m_goal = new TrapezoidProfile.State(position, 0); // new goal position
    m_setpoint =
        m_TrapezoidProfile.calculate(
            0.020, m_setpoint, m_goal); // calculates the new setpoint based on
    // the new goal

    positionVoltageController.Position = m_setpoint.position;
    positionVoltageController.Velocity = m_setpoint.velocity;
    tiltMotor.setControl(positionVoltageController);
  }

  public void tiltArmToSetPositionWPI(double position) {
    // if (!armManualControl) {
    // if (armPidController.calculate(tiltThroughBoreEncoder.get(), position) < 0 &&
    // bottomLimitSwitch.get()) {
    // tiltMotor.setControl(positionDutyCycleController.withOutput(
    // 0));
    // } else {
    // tiltMotor.setControl(positionDutyCycleController.withOutput(
    // armPidController.calculate(tiltThroughBoreEncoder.get(), position)));
    // }
    // }
    if (!armManualControl) {
      tiltMotor.setControl(
          positionDutyCycleController.withOutput(
              armPidController.calculate(tiltThroughBoreEncoder.get(), position)
                  * KArmFlipVelocity));
    }
  }

  public void setManualControl() {
    armManualControl = !armManualControl;
  }

  public boolean isAtSetPosition() {
    return Math.abs(m_setpoint.position - m_goal.position) <= KArmDeadZone; // need to tune
  }

  public double getSwerveMaxSpeed() {
    return speedModifier;
  }

  public void setSwerveMaxSpeed(double speedMod) {
    speedModifier = speedMod;
  }

  // will remind m_setpoint of current position using encoder
  // In case manual confuses the PID
  public void setPositionFromEncoder() {
    m_setpoint = new TrapezoidProfile.State(getTiltEncoder(), 0);
  }

  public double getTiltEncoder() {
    return tiltThroughBoreEncoder.get();
  }

  public double getArmSpeed() {
    return tiltMotor.getRotorVelocity().getValueAsDouble();
  }

  public void setMotorVoltage(Voltage volts) {
    tiltMotor.setControl(voltageController.withOutput(volts));
  }

  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //     return m_sysIdRoutineArm.quasistatic(direction);
  // }

  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //     return m_sysIdRoutineArm.dynamic(direction);
  // }

  public void stop() {
    tiltMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Cancoder", getTiltEncoder());

    SmartDashboard.putBoolean("Arm Manual", armManualControl);
    SmartDashboard.putNumber("Arm speed", getArmSpeed());
  }
}
