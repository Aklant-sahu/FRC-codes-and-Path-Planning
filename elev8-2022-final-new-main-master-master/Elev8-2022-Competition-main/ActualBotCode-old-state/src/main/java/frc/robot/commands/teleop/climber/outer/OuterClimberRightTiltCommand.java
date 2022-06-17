// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.climber.outer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climber.outer.OuterClimberSubsystem;

public class OuterClimberRightTiltCommand extends CommandBase {
  private OuterClimberSubsystem outerClimberSubsystem;

  /** Creates a new OuterClimberLeftTiltCommand. */
  public OuterClimberRightTiltCommand(OuterClimberSubsystem outerClimberSubsystem) {
    this.outerClimberSubsystem = outerClimberSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.outerClimberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.outerClimberSubsystem.setSpeed(new double[] { -0.4, 0.4 });
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.outerClimberSubsystem.setSpeed(new double[] { 0.0, 0.0 });
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !RobotContainer.joyD.getRawButton(2);
  }
}
