// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.auto.routine.AutonomousRoutineCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DrivingConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.auto.drive.tester.DriveByAFourthCoordinateCommand;
// import frc.robot.Constants.OIConstants.OIJoyC;
// import frc.robot.commands.auto.drive.AutonomousDriveRoutineGroupCommand;
// import frc.robot.commands.auto.drive.AutonomousTurnByAngleCommand;
import frc.robot.commands.auto.drive.tester.DriveByThirdCoordinateCommand;
// import frc.robot.commands.auto.shooter.ShooterByTimeCommand;
import frc.robot.commands.teleop.climber.inner.InnerClimberCommand;
import frc.robot.commands.teleop.climber.inner.InnerClimberLeftTiltCommand;
import frc.robot.commands.teleop.climber.inner.InnerClimberRightTiltCommand;
import frc.robot.commands.teleop.climber.outer.OuterClimberCommand;
import frc.robot.commands.teleop.climber.outer.OuterClimberHorizontalAlignmentCommand;
import frc.robot.commands.teleop.climber.outer.OuterClimberLeftTiltCommand;
import frc.robot.commands.teleop.climber.outer.OuterClimberRightTiltCommand;
import frc.robot.commands.teleop.climber.pg.inner.InnerPGClimberCommand;
import frc.robot.commands.teleop.climber.pg.inner.InnerPGClimberStopCommand;
import frc.robot.commands.teleop.climber.pg.outer.OuterPGClimberCommand;
import frc.robot.commands.teleop.climber.pg.outer.OuterPGClimberStopCommand;
import frc.robot.commands.teleop.drive.DriveCommand;
import frc.robot.commands.teleop.feeder.FeederCommand;
import frc.robot.commands.teleop.feeder.ServoFeederCommand;
// import frc.robot.commands.teleop.feeder.FeederCommand;
import frc.robot.commands.teleop.intake.IntakeCommand;
import frc.robot.commands.teleop.intake.IntakeStoppingCommand;
import frc.robot.commands.teleop.intake.ReverseCommand;
import frc.robot.commands.teleop.intakestarter.IntakeStartCommand;
import frc.robot.commands.teleop.shooter.ShooterCommand;
import frc.robot.subsystems.climber.inner.InnerClimberSubsystem;
import frc.robot.subsystems.climber.outer.OuterClimberSubsystem;
import frc.robot.subsystems.climber.pg.InnerPGSubsystem;
import frc.robot.subsystems.climber.pg.OuterPGSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeStarterSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.servo.ServoFeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // IO Devices

  public static Joystick joyD, joyC;
  public static NetworkTable table;
  public static NetworkTableEntry tv, tx, ty, ta;

  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final InnerClimberSubsystem innerClimberSubsystem;
  private final OuterClimberSubsystem outerClimberSubsystem;
  private final InnerPGSubsystem innerPGSubsystem;
  private final OuterPGSubsystem outerPGSubsystem;
  private final ServoFeederSubsystem servoFeederSubsystem;
  private final IntakeStarterSubsystem intakeStarterSubsystem;
  private final SlewRateLimiter speedLimit, turnLimit;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    this.driveSubsystem = new DriveSubsystem();
    this.intakeSubsystem = new IntakeSubsystem();
    this.feederSubsystem = new FeederSubsystem();
    this.shooterSubsystem = new ShooterSubsystem();
    this.innerClimberSubsystem = new InnerClimberSubsystem();
    this.outerClimberSubsystem = new OuterClimberSubsystem();
    this.innerPGSubsystem = new InnerPGSubsystem();
    this.outerPGSubsystem = new OuterPGSubsystem();
    this.servoFeederSubsystem = new ServoFeederSubsystem();
    this.intakeStarterSubsystem = new IntakeStarterSubsystem();

    this.speedLimit = new SlewRateLimiter(DrivingConstants.kRiseLimiter);
    this.turnLimit = new SlewRateLimiter(DrivingConstants.kRiseLimiter);

    RobotContainer.joyD = new Joystick(OIConstants.kDriverJoystickPort);
    RobotContainer.joyC = new Joystick(OIConstants.kClimberJoystickPort);
    RobotContainer.table = NetworkTableInstance.getDefault().getTable(VisionConstants.limelight);
    RobotContainer.tv = RobotContainer.table.getEntry(VisionConstants.tv);
    RobotContainer.tx = RobotContainer.table.getEntry(VisionConstants.tx);
    RobotContainer.ty = RobotContainer.table.getEntry(VisionConstants.ty);
    RobotContainer.ta = RobotContainer.table.getEntry(VisionConstants.ta);
    // Configure the button bindings
    configureButtonBindings();

    // Set Default commands
    // Intake Start
    this.intakeStarterSubsystem.setDefaultCommand(
        new IntakeStartCommand(this.intakeStarterSubsystem, () -> dpadButtonUp_JoyD(), () -> dpadButtonDown_JoyD()));
    ;

    // Drive
    this.driveSubsystem.setDefaultCommand(
        new DriveCommand(this.driveSubsystem, () -> -1 * RobotContainer.joyD.getRawAxis(OIConstants.kJoyDSpeedAxis),
            () -> RobotContainer.joyD.getRawAxis(2), this.speedLimit, this.turnLimit));

    // Intake
    this.intakeSubsystem.setDefaultCommand(new IntakeStoppingCommand(this.intakeSubsystem));

    // Shooter
    // this.shooterSubsystem.setDefaultCommand(new
    // ShooterJoyTestCommand(this.shooterSubsystem, () ->
    // RobotContainer.joyD.getRawAxis(3)));

    // PG - Inner
    this.innerPGSubsystem.setDefaultCommand(new InnerPGClimberCommand(this.innerPGSubsystem,
        () -> RobotContainer.joyC.getRawAxis(OIConstants.OIJoyC.innerPG_Axis_Two),
        () -> RobotContainer.joyC.getRawButton(OIConstants.OIJoyC.innerPG_Button_Five)));

    // PG - Outer
    this.outerPGSubsystem.setDefaultCommand(new OuterPGClimberCommand(this.outerPGSubsystem,
        () -> RobotContainer.joyC.getRawAxis(OIConstants.OIJoyC.outerPG_Axis_Three),
        () -> RobotContainer.joyC.getRawButton(OIConstants.OIJoyC.outerPG_Button_Six)));

    // Climber - Inner
    this.innerClimberSubsystem.setDefaultCommand(
        new InnerClimberCommand(this.innerClimberSubsystem,
            () -> getThresholledAxis(RobotContainer.joyC, OIConstants.OIJoyC.innerClimber_Axis_One, 0.2)));

    // Climber - Outer
    this.outerClimberSubsystem.setDefaultCommand(
        new OuterClimberCommand(this.outerClimberSubsystem,
            () -> getThresholledAxis(RobotContainer.joyC, OIConstants.OIJoyC.outerClimber_Axis_Five, 0.2)));

  }

  // double getActualAngle() {
  // return (-RobotContainer.navx.getAngle() + RobotContainer.GYRO_OFFSET);
  // }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   * 
   */

  public double getThresholledAxis(Joystick joy, int axisNumber, double threshold) {
    return (Math.abs(joy.getRawAxis(axisNumber)) < threshold ? 0
        : joy.getRawAxis(axisNumber));
  }

  private void configureButtonBindings() {
    // Feeder Button Integration
    // new JoystickButton(RobotContainer.joyD, OIConstants.feeder_X_ButtonNumber)
    // .whenActive(new FeederCommand(this.feederSubsystem));

    // Intake Forward Button Integration
    new JoystickButton(RobotContainer.joyC, 7)
        .toggleWhenActive(new IntakeCommand(this.intakeSubsystem)); // X
    new JoystickButton(RobotContainer.joyD, 7)
        .toggleWhenActive(new ReverseCommand(this.intakeSubsystem, this.feederSubsystem)); // A

    // Feeder Button
    new JoystickButton(RobotContainer.joyD, 5)
        .toggleWhenActive(new FeederCommand(this.feederSubsystem)); // B

    // Feeder Servo Button
    new JoystickButton(RobotContainer.joyD, 8)
        .toggleWhenActive(new ServoFeederCommand(this.servoFeederSubsystem)); // Y

    // Shooter Button Binding Integration [by Time] => Works
    // new JoystickButton(RobotContainer.joyD, OIConstants.shooter_RB_ButtonNumber)
    // .whenActive(new ShooterByTimeCommand(shooterSubsystem, 10));

    // Shooter Button Binding Integration
    new JoystickButton(RobotContainer.joyD, 6)
        .toggleWhenActive(new ShooterCommand(this.shooterSubsystem));

    // PG Stopper Button Binding Integration - Inner
    new JoystickButton(RobotContainer.joyC,
        OIConstants.OIJoyC.innerPGStop_Button_Two)
        .whenPressed(new InnerPGClimberStopCommand(this.innerPGSubsystem,
            this.innerPGSubsystem.getInnerPGPosition(),
            () -> dpadButtonRight()));

    // PG Stopper Button Binding Integration - Outer
    new JoystickButton(RobotContainer.joyC,
        OIConstants.OIJoyC.outerPGStop_Button_Three)
        .whenPressed(new OuterPGClimberStopCommand(this.outerPGSubsystem,
            this.outerPGSubsystem.getOuterPGPosition(), () -> dpadButtonLeft()));

    // Bot Goal Aligning Tool
    new JoystickButton(RobotContainer.joyD, 9)
        .whenPressed(
            new DriveByThirdCoordinateCommand(driveSubsystem, -2.21, 0));

    // Bot Goal - Align A Meter Away
    new JoystickButton(RobotContainer.joyD, 10)
        .whenPressed(
            new DriveByAFourthCoordinateCommand(driveSubsystem, -2.21, 0, 1));

    new JoystickButton(RobotContainer.joyD, 11).whenPressed(new SequentialCommandGroup(
        new DriveByAFourthCoordinateCommand(driveSubsystem, -2.21, 0, 1),
        new DriveByThirdCoordinateCommand(driveSubsystem, -2.21, 0)));

    // Bot Distance aligning tool to the Goal

    // new DriveBySecondCoordinateCommand(this.driveSubsystem, 180));

    // Test Autonomous Angle Movement
    // new JoystickButton(RobotContainer.joyD, 5)
    // .whenPressed(new DriveBySecondCoordinateCommand(this.driveSubsystem,30));

    // Climber Body Horizontal Alignment - Inner
    // new JoystickButton(RobotContainer.joyC, 1)
    // .whenPressed(new
    // InnerClimberHorizontalAlignmentCommand(this.innerClimberSubsystem));

    // Climber Body Horizontal Alignment - Outer
    new JoystickButton(RobotContainer.joyC, 4)
        .whenPressed(new OuterClimberHorizontalAlignmentCommand(this.outerClimberSubsystem));

    // Climber Left Tilt - Inner
    new JoystickButton(RobotContainer.joyD, 3).whenPressed(new InnerClimberLeftTiltCommand(this.innerClimberSubsystem));

    // Climber Right Tilt - Inner
    new JoystickButton(RobotContainer.joyD, 4)
        .whenPressed(new InnerClimberRightTiltCommand(this.innerClimberSubsystem));

    // Climber Left Tilt - Outer
    new JoystickButton(RobotContainer.joyD, 1).whenPressed(new OuterClimberLeftTiltCommand(this.outerClimberSubsystem));

    // Climber Right Tilt - Outer
    new JoystickButton(RobotContainer.joyD, 2)
        .whenPressed(new OuterClimberRightTiltCommand(this.outerClimberSubsystem));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return new AutonomousDriveRoutineGroupCommand(this.driveSubsystem);
    // return new InnerClimberHorizontalAlignmentCommand(innerClimberSubsystem);
    return new AutonomousRoutineCommand(shooterSubsystem, feederSubsystem, driveSubsystem, intakeSubsystem);
  }

  public static boolean getTarget() {
    RobotContainer.tv = RobotContainer.table.getEntry(VisionConstants.tv);
    return tv.getBoolean(false);
  }

  public static double getDistanceToGoal() {
    RobotContainer.ty = RobotContainer.table.getEntry(VisionConstants.ty);
    return ty.getDouble(VisionConstants.defaultValue);
  }

  public static double getDistanceGyroSeparationFromGoal() {
    RobotContainer.tx = RobotContainer.table.getEntry(VisionConstants.tx);
    return tx.getDouble(VisionConstants.defaultValue);
  }

  public double getAreaOfGoal() {
    RobotContainer.ta = RobotContainer.table.getEntry(VisionConstants.ta);
    return ta.getDouble(VisionConstants.defaultAreaValue);
  }

  public boolean dpadButtonRight() {
    return (RobotContainer.joyC.getPOV() >= 45 && RobotContainer.joyC.getPOV() <= 135);
  }

  public boolean dpadButtonLeft() {
    return (RobotContainer.joyC.getPOV() >= 225 && RobotContainer.joyC.getPOV() <= 315);
  }

  public boolean dpadButtonUp() {
    return (RobotContainer.joyC.getPOV() >= 315 && RobotContainer.joyC.getPOV() < 360)
        || (RobotContainer.joyC.getPOV() >= 0 && RobotContainer.joyC.getPOV() <= 45);
  }

  public boolean dpadButtonDown() {
    return (RobotContainer.joyC.getPOV() >= 135 && RobotContainer.joyC.getPOV() <= 225);
  }

  public boolean dpadButtonRight_JoyD() {
    return (RobotContainer.joyC.getPOV() >= 45 && RobotContainer.joyC.getPOV() <= 135);
  }

  public boolean dpadButtonLeft_JoyD() {
    return (RobotContainer.joyC.getPOV() >= 225 && RobotContainer.joyC.getPOV() <= 315);
  }

  public boolean dpadButtonUp_JoyD() {
    return (RobotContainer.joyC.getPOV() >= 315 && RobotContainer.joyC.getPOV() < 360)
        || (RobotContainer.joyC.getPOV() >= 0 && RobotContainer.joyC.getPOV() <= 45);
  }

  public boolean dpadButtonDown_JoyD() {
    return (RobotContainer.joyC.getPOV() >= 135 && RobotContainer.joyC.getPOV() <= 225);
  }
}
