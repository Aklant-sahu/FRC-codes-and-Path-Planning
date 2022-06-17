// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class ReverseCommand extends CommandBase {
  /** Creates a new ReverseIntakeCommand. */
  private IntakeSubsystem intakeSubsystem;
  private FeederSubsystem feederSubsystem;

  public ReverseCommand(IntakeSubsystem intakeSubsystem, FeederSubsystem feederSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.feederSubsystem = feederSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.intakeSubsystem.setIntakeSpeed(0.55);
    this.feederSubsystem.setFeederSpeed(-0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.intakeSubsystem.setIntakeSpeed(IntakeConstants.stopSpeed);
    this.feederSubsystem.setFeederSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !RobotContainer.joyD.getRawButton(7);
  }
}
