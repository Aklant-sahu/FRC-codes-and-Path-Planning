// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
  private IntakeSubsystem intakeSubsystem;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.intakeSubsystem.setIntakeSpeed(IntakeConstants.flowSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.intakeSubsystem.setIntakeSpeed(IntakeConstants.stopSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return !RobotContainer.joyD.getRawButton(OIConstants.intakeForward_Y_ButtonNumber);
    return false;
  }
}
