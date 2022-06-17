// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.climber.inner;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climber.inner.InnerClimberSubsystem;

public class InnerClimberRightTiltCommand extends CommandBase {
  private InnerClimberSubsystem innerClimberSubsystem;

  /** Creates a new InnerClimberRightTiltCommand. */
  public InnerClimberRightTiltCommand(InnerClimberSubsystem innerClimberSubsystem) {
    this.innerClimberSubsystem = innerClimberSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.innerClimberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.innerClimberSubsystem.setSpeed(new double[] { -0.4, 0.4 });
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.innerClimberSubsystem.setSpeed(new double[] { 0.0, 0.0 });
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !RobotContainer.joyD.getRawButton(4);
  }
}
