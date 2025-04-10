// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.ArmConstants.ArmPositionConstants.*;
import static frc.robot.Constants.CoralIntakeConstants.*;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.LiftConstants.*;
import static frc.robot.Constants.LiftConstants.LiftPositionConstants.*;
import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.CommandGroups.AutoDrive.AutoScore;
import frc.robot.CommandGroups.LiftSetpoints.LiftandArmIntake;
import frc.robot.CommandGroups.LiftSetpoints.LiftandArmTier1;
import frc.robot.CommandGroups.LiftSetpoints.LiftandArmTier2;
import frc.robot.CommandGroups.LiftSetpoints.LiftandArmTier3;
import frc.robot.CommandGroups.LiftSetpoints.LiftandArmTier4;
import frc.robot.commands.Arm.MoveArmStow;
import frc.robot.commands.Arm.SetArmManualMode;
import frc.robot.commands.Arm.TiltArmManually;
import frc.robot.commands.Arm.TiltArmToSetPosition;
import frc.robot.commands.Coral.CoralDefault;
import frc.robot.commands.Coral.SpinCoralIntake;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Hang.MoveHang;
import frc.robot.commands.Hang.MoveHangServo;
import frc.robot.commands.Lift.MoveLift;
import frc.robot.commands.Lift.MoveLiftToPos;
import frc.robot.commands.Lift.SetLiftManualMode;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Hang;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.SubsystemUtil;
import java.io.IOException;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drive drive;
  private final Vision vision;
  public final Arm arm;
  public final Lift lift;
  public final CoralIntake coralIntake;
  public final SubsystemUtil subsystemUtil;
  public final Hang hang;

  // Commands

  public final Command baseTurboMode;
  public final Command baseNormalMode;
  public final Command baseSlowMode;

  public final MoveLift liftStop;
  public final MoveLift moveLiftUp;
  public final MoveLift moveLiftDown;
  public final MoveLiftToPos liftStow;
  public final SetLiftManualMode setLiftManualMode;

  public final TiltArmManually armStop;
  public final TiltArmManually tiltArmManuallyUp;
  public final TiltArmManually tiltArmManuallyDown;
  public final MoveArmStow armStow;
  public final TiltArmToSetPosition armtopos;
  public final SetArmManualMode setArmManualMode;

  public final SpinCoralIntake stopCoralIntake;
  public final CoralDefault coralDefault;
  public final SpinCoralIntake spinCoralIntakeForward;
  public final SpinCoralIntake spinCoralIntakeForwardSlow;
  public final SpinCoralIntake spinCoralIntakeBackward;

  public final MoveHang moveHangUp;
  public final MoveHang moveHangDown;
  public final MoveHang moveHangStop;

  public final MoveHangServo lockRachet;
  public final MoveHangServo unlockRachet;

  // Command Groups
  public final LiftandArmTier4 liftandArmTier4;
  public final LiftandArmTier3 liftandArmTier3;
  public final LiftandArmTier2 liftandArmTier2;
  public final LiftandArmTier1 liftandArmTier1;
  public final LiftandArmIntake liftandArmIntake;

  /* Setting up bindings for necessary control of the swerve drive platform */
  // private final SwerveRequest.SwerveDriveBrake brake = new
  // SwerveRequest.SwerveDriveBrake();
  // private final SwerveRequest.PointWheelsAt point = new
  // SwerveRequest.PointWheelsAt();

  public static Joystick logitech;
  public static Joystick compStreamDeck;
  public static Joystick testStreamDeck;
  public static Joystick autonTestStreamDeck;
  // public final CommandXboxController joystick = new CommandXboxController(1);
  public JoystickButton logitechBtnX,
      logitechBtnA,
      logitechBtnB,
      logitechBtnY,
      logitechBtnLB,
      logitechBtnRB,
      logitechBtnLT,
      logitechBtnRT,
      logitechBtnBack,
      logitechBtnStart; // Logitech Button
  public JoystickButton compStreamDeck1,
      compStreamDeck2,
      compStreamDeck3,
      compStreamDeck4,
      compStreamDeck5,
      compStreamDeck6,
      compStreamDeck7,
      compStreamDeck8,
      compStreamDeck9,
      compStreamDeck10,
      compStreamDeck11,
      compStreamDeck12,
      compStreamDeck13,
      compStreamDeck14,
      compStreamDeck15,
      compStreamDeck16,
      compStreamDeck17,
      compStreamDeck18,
      compStreamDeck19;

  // Top Left SD = 1, numbered from left to right
  public JoystickButton testStreamDeck1,
      testStreamDeck2,
      testStreamDeck3,
      testStreamDeck4,
      testStreamDeck5,
      testStreamDeck6,
      testStreamDeck7,
      testStreamDeck8,
      testStreamDeck9, // Vjoy 2
      testStreamDeck10,
      testStreamDeck11,
      testStreamDeck12,
      testStreamDeck13,
      testStreamDeck14,
      testStreamDeck15;
  public JoystickButton autonTestStreamDeck1,
      autonTestStreamDeck2,
      autonTestStreamDeck3,
      autonTestStreamDeck4,
      autonTestStreamDeck5,
      autonTestStreamDeck6,
      autonTestStreamDeck7,
      autonTestStreamDeck8,
      autonTestStreamDeck9, // Vjoy 2
      autonTestStreamDeck10,
      autonTestStreamDeck11,
      autonTestStreamDeck12,
      autonTestStreamDeck13,
      autonTestStreamDeck14,
      autonTestStreamDeck15;

  private double baseSpeed = KBaseNormalMode;
  public final AutoScore Right12L4,
      Right10L4,
      Right8L4,
      Right6L4,
      Right4L4,
      Right2L4,
      Left12L4,
      Left10L4,
      Left8L4,
      Left6L4,
      Left4L4,
      Left2L4;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Subsystems
    arm = new Arm();
    lift = new Lift();
    coralIntake = new CoralIntake();
    subsystemUtil = new SubsystemUtil(arm);
    hang = new Hang();

    liftStop = new MoveLift(lift, KLiftStopVelocity);
    moveLiftUp = new MoveLift(lift, KLiftMoveVelocity);
    moveLiftDown = new MoveLift(lift, -KLiftMoveVelocity);
    liftStow = new MoveLiftToPos(lift, KLiftPositionStow);
    setLiftManualMode = new SetLiftManualMode(lift);

    armStop = new TiltArmManually(arm, KArmStopVelocity);
    tiltArmManuallyUp = new TiltArmManually(arm, KArmMoveVelocity);
    tiltArmManuallyDown = new TiltArmManually(arm, -KArmMoveVelocity);
    armtopos = new TiltArmToSetPosition(arm, 50);
    setArmManualMode = new SetArmManualMode(arm);
    armStow = new MoveArmStow(arm, KArmPositionStow);

    stopCoralIntake = new SpinCoralIntake(coralIntake, 0);
    coralDefault = new CoralDefault(coralIntake, subsystemUtil);

    spinCoralIntakeForward = new SpinCoralIntake(coralIntake, 0.75);
    spinCoralIntakeForwardSlow = new SpinCoralIntake(coralIntake, 0.5);
    spinCoralIntakeBackward = new SpinCoralIntake(coralIntake, -KCoralIntakeSpeed);

    moveHangUp = new MoveHang(hang, 1);
    moveHangDown = new MoveHang(hang, -1);
    moveHangStop = new MoveHang(hang, 0);

    lockRachet = new MoveHangServo(hang, 0);
    unlockRachet = new MoveHangServo(hang, 1);

    // Command Groups
    liftandArmTier4 = new LiftandArmTier4(arm, lift);
    liftandArmTier3 = new LiftandArmTier3(arm, lift);
    liftandArmTier2 = new LiftandArmTier2(arm, lift);
    liftandArmTier1 = new LiftandArmTier1(arm, lift);
    liftandArmIntake = new LiftandArmIntake(arm, lift);

    Right12L4 = new AutoScore("Right12L4", lift, arm);
    Right10L4 = new AutoScore("Right10L4", lift, arm);
    Right8L4 = new AutoScore("Right8L4", lift, arm);
    Right6L4 = new AutoScore("Right6L4", lift, arm);
    Right4L4 = new AutoScore("Right4L4", lift, arm);
    Right2L4 = new AutoScore("Right2L4", lift, arm);
    Left12L4 = new AutoScore("Left12L4", lift, arm);
    Left10L4 = new AutoScore("Left10L4", lift, arm);
    Left8L4 = new AutoScore("Left8L4", lift, arm);
    Left6L4 = new AutoScore("Left6L4", lift, arm);
    Left4L4 = new AutoScore("Left4L4", lift, arm);
    Left2L4 = new AutoScore("Left2L4", lift, arm);

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                // new VisionIOLimelight(camera0Name, drive::getRotation),
                new VisionIOLimelight(camera1Name, drive::getRotation));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                // new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        break;
    }
    baseNormalMode = DriveCommands.changesSpeedFactor(drive, KBaseNormalMode);
    baseSlowMode = DriveCommands.changesSpeedFactor(drive, KBaseSlowMode);
    baseTurboMode = DriveCommands.changesSpeedFactor(drive, KBaseTurboMode);

    SmartDashboard.setDefaultBoolean("ReleaseCoral?", false);

    // SmartDashboard.putData("Swerve Drive", new Sendable() {
    // @Override
    // public void initSendable(SendableBuilder builder) {
    // builder.setSmartDashboardType("SwerveDrive");

    // // Swerve 1
    // builder.addDoubleProperty("Front Left Angle",
    // () -> Swerve.get().getModule(0).getCurrentState().angle.getDegrees(),
    // null);
    // builder.addDoubleProperty("Front Left Velocity",
    // () -> Swerve.get().getModule(0).getCurrentState().speedMetersPerSecond,
    // null);

    // // Swerve 2
    // builder.addDoubleProperty("Front Right Angle",
    // () -> Swerve.get().getModule(1).getCurrentState().angle.getDegrees(),
    // null);
    // builder.addDoubleProperty("Front Right Velocity",
    // () -> Swerve.get().getModule(1).getCurrentState().speedMetersPerSecond,
    // null);

    // // Swerve 3
    // builder.addDoubleProperty("Back Left Angle",
    // () -> Swerve.get().getModule(2).getCurrentState().angle.getDegrees(),
    // null);
    // builder.addDoubleProperty("Back Left Velocity",
    // () -> Swerve.get().getModule(2).getCurrentState().speedMetersPerSecond,
    // null);

    // // Swerve 4
    // builder.addDoubleProperty("Back Right Angle",
    // () -> Swerve.get().getModule(3).getCurrentState().angle.getDegrees(),
    // null);
    // builder.addDoubleProperty("Back Right Velocity",
    // () -> Swerve.get().getModule(3).getCurrentState().speedMetersPerSecond,
    // null);

    // // Rotation
    // builder.addDoubleProperty("Robot Angle", () ->
    // Swerve.get().getRotation3d().toRotation2d().getDegrees(),
    // null);

    // }
    // });

    NamedCommands.registerCommand("LiftT4", liftandArmTier4);
    NamedCommands.registerCommand("CoralOut", spinCoralIntakeBackward);
    NamedCommands.registerCommand("CoralIn", spinCoralIntakeForward);
    NamedCommands.registerCommand("CoralInSlow", spinCoralIntakeForwardSlow);
    NamedCommands.registerCommand("ArmIntakePos", liftandArmIntake);
    NamedCommands.registerCommand("LiftStow", liftStow);
    NamedCommands.registerCommand("ArmStow", armStow);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // DS Ports
    logitech = new Joystick(KLogitechPort); // Logitech Dual Action
    compStreamDeck = new Joystick(KCompStreamDeckPort); // Stream Deck + vjoy
    testStreamDeck = new Joystick(KTestingStreamDeckPort); // Stream Deck + vjoy
    autonTestStreamDeck = new Joystick(KAutonTestingStreamDeckPort); // Stream Deck + vjoy

    // Logitch Buttons
    logitechBtnX = new JoystickButton(logitech, KLogitechButtonX);
    logitechBtnA = new JoystickButton(logitech, KLogitechButtonA);
    logitechBtnB = new JoystickButton(logitech, KLogitechButtonB);
    logitechBtnY = new JoystickButton(logitech, KLogitechButtonY);
    logitechBtnLB = new JoystickButton(logitech, KLogitechLeftBumper);
    logitechBtnRB = new JoystickButton(logitech, KLogitechRightBumper);
    logitechBtnLT = new JoystickButton(logitech, KLogitechLeftTrigger);
    logitechBtnRT = new JoystickButton(logitech, KLogitechRightTrigger);
    logitechBtnBack = new JoystickButton(logitech, KLogitechBtnBack);
    logitechBtnStart = new JoystickButton(logitech, KLogitechRightStart);

    // Streamdeck Pages used in match
    compStreamDeck1 = new JoystickButton(compStreamDeck, 1);
    compStreamDeck2 = new JoystickButton(compStreamDeck, 2);
    compStreamDeck3 = new JoystickButton(compStreamDeck, 3);
    compStreamDeck4 = new JoystickButton(compStreamDeck, 4);
    compStreamDeck5 = new JoystickButton(compStreamDeck, 5);
    compStreamDeck6 = new JoystickButton(compStreamDeck, 6);
    compStreamDeck7 = new JoystickButton(compStreamDeck, 7);
    compStreamDeck8 = new JoystickButton(compStreamDeck, 8);
    compStreamDeck9 = new JoystickButton(compStreamDeck, 9);
    compStreamDeck10 = new JoystickButton(compStreamDeck, 10);
    compStreamDeck11 = new JoystickButton(compStreamDeck, 11);
    compStreamDeck12 = new JoystickButton(compStreamDeck, 12);
    compStreamDeck13 = new JoystickButton(compStreamDeck, 13);
    compStreamDeck14 = new JoystickButton(compStreamDeck, 14);
    compStreamDeck15 = new JoystickButton(compStreamDeck, 15);
    compStreamDeck16 = new JoystickButton(compStreamDeck, 16);
    compStreamDeck17 = new JoystickButton(compStreamDeck, 17);
    compStreamDeck18 = new JoystickButton(compStreamDeck, 18);
    compStreamDeck19 = new JoystickButton(compStreamDeck, 19);

    // Streamdeck Pages used for testing
    testStreamDeck1 = new JoystickButton(testStreamDeck, 1);
    testStreamDeck2 = new JoystickButton(testStreamDeck, 2);
    testStreamDeck3 = new JoystickButton(testStreamDeck, 3);
    testStreamDeck4 = new JoystickButton(testStreamDeck, 4);
    testStreamDeck5 = new JoystickButton(testStreamDeck, 5);
    testStreamDeck6 = new JoystickButton(testStreamDeck, 6);
    testStreamDeck7 = new JoystickButton(testStreamDeck, 7);
    testStreamDeck8 = new JoystickButton(testStreamDeck, 8);
    testStreamDeck9 = new JoystickButton(testStreamDeck, 9);
    testStreamDeck10 = new JoystickButton(testStreamDeck, 10);
    testStreamDeck11 = new JoystickButton(testStreamDeck, 11);
    testStreamDeck12 = new JoystickButton(testStreamDeck, 12);
    testStreamDeck13 = new JoystickButton(testStreamDeck, 13);
    testStreamDeck14 = new JoystickButton(testStreamDeck, 14);
    testStreamDeck15 = new JoystickButton(testStreamDeck, 15);

    autonTestStreamDeck1 = new JoystickButton(testStreamDeck, 1);
    autonTestStreamDeck2 = new JoystickButton(testStreamDeck, 2);
    autonTestStreamDeck3 = new JoystickButton(testStreamDeck, 3);
    autonTestStreamDeck4 = new JoystickButton(testStreamDeck, 4);
    autonTestStreamDeck5 = new JoystickButton(testStreamDeck, 5);
    autonTestStreamDeck6 = new JoystickButton(testStreamDeck, 6);
    autonTestStreamDeck7 = new JoystickButton(testStreamDeck, 7);
    autonTestStreamDeck8 = new JoystickButton(testStreamDeck, 9);
    autonTestStreamDeck9 = new JoystickButton(testStreamDeck, 9);
    autonTestStreamDeck10 = new JoystickButton(testStreamDeck, 10);
    autonTestStreamDeck11 = new JoystickButton(testStreamDeck, 11);
    autonTestStreamDeck12 = new JoystickButton(testStreamDeck, 12);
    autonTestStreamDeck13 = new JoystickButton(testStreamDeck, 13);
    autonTestStreamDeck14 = new JoystickButton(testStreamDeck, 14);
    autonTestStreamDeck15 = new JoystickButton(testStreamDeck, 15);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    // logitechBtnLB.whileTrue(baseTurboMode);
    // logitechBtnLT.whileTrue(baseSlowMode);

    // // When either button is released, it sets it equal to normal mode
    // logitechBtnLB.whileFalse(baseNormalMode);
    // logitechBtnLT.onFalse(baseNormalMode);

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> getLogiLeftYAxis(), () -> getLogiLeftXAxis(), () -> getLogiRightXAxis()));

    arm.setDefaultCommand(armStow);
    lift.setDefaultCommand(liftStow);
    // coralIntake.setDefaultCommand(armStow);
    coralIntake.setDefaultCommand(coralDefault); // could be an issue
    hang.setDefaultCommand(moveHangStop);
    logitechBtnRB.whileTrue(spinCoralIntakeForward);
    logitechBtnRT.whileTrue(spinCoralIntakeBackward);
    // Lock to 0° when A button is held
    logitechBtnA.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive, () -> getLogiLeftYAxis(), () -> getLogiLeftXAxis(), () -> new Rotation2d()));
    // Lock to 0° when A button is held
    logitechBtnLT.whileTrue(
        DriveCommands.joystickDrive(
            drive,
            () -> getLogiLeftYAxis() * 0.5,
            () -> getLogiLeftXAxis() * 0.5,
            () -> getLogiRightXAxis() * 0.5));

    // Reset gyro to 0° when Y button is pressed
    logitechBtnY.onTrue(
        Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                drive)
            .ignoringDisable(true));

    logitechBtnB
        .and(compStreamDeck7)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> getLogiLeftYAxis(),
                () -> getLogiLeftXAxis(),
                () -> getAngleFromReef(12)));
    logitechBtnX
        .and(compStreamDeck7)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> getLogiLeftYAxis(),
                () -> getLogiLeftXAxis(),
                () -> getAngleFromReef(1)));
    logitechBtnB
        .and(compStreamDeck12)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> getLogiLeftYAxis(),
                () -> getLogiLeftXAxis(),
                () -> getAngleFromReef(2)));
    logitechBtnX
        .and(compStreamDeck12)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> getLogiLeftYAxis(),
                () -> getLogiLeftXAxis(),
                () -> getAngleFromReef(3)));
    logitechBtnB
        .and(compStreamDeck11)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> getLogiLeftYAxis(),
                () -> getLogiLeftXAxis(),
                () -> getAngleFromReef(4)));
    logitechBtnX
        .and(compStreamDeck11)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> getLogiLeftYAxis(),
                () -> getLogiLeftXAxis(),
                () -> getAngleFromReef(5)));
    logitechBtnB
        .and(compStreamDeck6)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> getLogiLeftYAxis(),
                () -> getLogiLeftXAxis(),
                () -> getAngleFromReef(6)));
    logitechBtnX
        .and(compStreamDeck6)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> getLogiLeftYAxis(),
                () -> getLogiLeftXAxis(),
                () -> getAngleFromReef(7)));
    logitechBtnB
        .and(compStreamDeck1)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> getLogiLeftYAxis(),
                () -> getLogiLeftXAxis(),
                () -> getAngleFromReef(8)));
    logitechBtnX
        .and(compStreamDeck1)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> getLogiLeftYAxis(),
                () -> getLogiLeftXAxis(),
                () -> getAngleFromReef(9)));
    logitechBtnB
        .and(compStreamDeck2)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> getLogiLeftYAxis(),
                () -> getLogiLeftXAxis(),
                () -> getAngleFromReef(10)));
    logitechBtnX
        .and(compStreamDeck2)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> getLogiLeftYAxis(),
                () -> getLogiLeftXAxis(),
                () -> getAngleFromReef(11)));

    // Lift and Arm Setpoints
    compStreamDeck3.whileTrue(liftandArmIntake);
    compStreamDeck4.whileTrue(liftandArmTier4);
    compStreamDeck9.whileTrue(liftandArmTier3);
    compStreamDeck14.whileTrue(liftandArmTier2);
    // compStreamDeck11.whileTrue(liftandArmTier1);

    // Coral Intake
    compStreamDeck5.whileTrue(spinCoralIntakeForward);
    compStreamDeck10.whileTrue(spinCoralIntakeBackward);

    compStreamDeck8.whileTrue(spinCoralIntakeForward);
    compStreamDeck13.whileTrue(spinCoralIntakeBackward);

    // // Manual Modes
    // compStreamDeck13.onTrue(setArmManualMode);
    // compStreamDeck14.onTrue(setLiftManualMode);

    // compStreamDeck3.whileTrue(tiltArmManuallyUp);
    // compStreamDeck8.whileTrue(tiltArmManuallyDown);
    // compStreamDeck4.whileTrue(moveLiftUp);
    // compStreamDeck9.whileTrue(moveLiftDown);

    compStreamDeck15.whileTrue(lockRachet);
    compStreamDeck16.whileTrue(unlockRachet);
    compStreamDeck17.whileTrue(moveHangUp);
    compStreamDeck18.whileTrue(moveHangDown);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public double getLogiRightYAxis() {
    final double Y = logitech.getRawAxis(KRightYAxis);
    SmartDashboard.putNumber("getLogiRightYAxis", -Y);
    if (Y > KDeadZone || Y < -KDeadZone) return -Y;
    else return 0;
  }

  public double getLogiLeftYAxis() {
    final double Y = logitech.getY();
    SmartDashboard.putNumber("getLogiLeftYAxis", -Y);
    if (Y > KDeadZone || Y < -KDeadZone) return -Y;
    else return 0;
  }

  public double getLogiRightXAxis() {
    double X = logitech.getZ();
    SmartDashboard.putNumber("getLogiRightXAxis", -X);
    if (X > KDeadZone || X < -KDeadZone) {
      return -X;
    } else {
      return 0;
    }
  }

  public double getLogiLeftXAxis() {
    double X = logitech.getX();
    SmartDashboard.putNumber("getLogiLeftXAxis", -X);
    if (X > KDeadZone || X < -KDeadZone) {
      return -X;
    } else {
      return 0;
    }
  }

  public void reefReleasePoint(int stakeNumber, DriverStation.Alliance allianceColor) {
    if (allianceColor == DriverStation.Alliance.Blue) {
      double BDifX = Math.pow(KBlueStakes[stakeNumber - 1].getX() - drive.getPose().getX(), 2);
      double BDifY = Math.pow(KBlueStakes[stakeNumber - 1].getY() - drive.getPose().getY(), 2);
      double BDif = Math.sqrt(BDifX + BDifY);
      SmartDashboard.putNumber("BDif", BDif);
      if (BDif < 0.65 && BDif > 0.3) {
        SmartDashboard.putBoolean("ReleaseCoral?", true);
      } else {
        SmartDashboard.putBoolean("ReleaseCoral?", false);
      }
    }
  }

  public Rotation2d getAngleFromReef(int stakeNumber) {
    if (DriverStation.getAlliance().isPresent()) {
      reefReleasePoint(stakeNumber, DriverStation.getAlliance().get());
      return getAngleFromReef(DriverStation.getAlliance().get(), drive.getPose(), stakeNumber);
    }
    return drive.getRotation();
  }

  public static Rotation2d getAngleFromReef(
      DriverStation.Alliance allianceColor, Pose2d pose, int stakeNumber) {
    double theta = 0;
    double angle = 0;
    double lambda = pose.getRotation().getDegrees();
    double xPos = pose.getX();
    double yPos = pose.getY();
    double BstakeXpose = KBlueStakes[stakeNumber - 1].getX();
    double BstakeYpose = KBlueStakes[stakeNumber - 1].getY();
    double RstakeXpose = KRedStakes[stakeNumber - 1].getX();
    double RstakeYpose = KRedStakes[stakeNumber - 1].getY();

    if (allianceColor == DriverStation.Alliance.Blue) {
      if (Math.abs(xPos - BstakeXpose) < 0.01) {
        return pose.getRotation();
      }
      theta = (Math.atan((yPos - BstakeYpose) / (xPos - BstakeXpose)) * (180 / Math.PI));
      angle = theta - lambda;
    } else {
      if (Math.abs(xPos - RstakeXpose) < 0.01) {
        return pose.getRotation();
      }
      theta = (Math.atan((yPos - RstakeYpose) / (xPos - RstakeXpose)) * (180 / Math.PI));
      angle = theta - lambda;
    }
    SmartDashboard.putNumber("angletoRot", angle);
    SmartDashboard.putNumber("angletoRot2", pose.getRotation().getDegrees() - angle);
    // if ((angle) < 2 && angle > -2) {
    // return pose.getRotation();
    // }
    SmartDashboard.putString(
        "finalrotationReturn",
        pose.getRotation().rotateBy(new Rotation2d(Math.toRadians(angle))).toString());
    return pose.getRotation().rotateBy(new Rotation2d(Math.toRadians(angle)));
  }
}
