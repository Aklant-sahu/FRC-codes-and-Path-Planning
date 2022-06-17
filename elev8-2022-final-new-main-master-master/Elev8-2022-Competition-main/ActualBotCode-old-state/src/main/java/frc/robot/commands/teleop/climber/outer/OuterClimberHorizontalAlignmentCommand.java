// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.climber.outer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.outer.OuterClimberSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;

public class OuterClimberHorizontalAlignmentCommand extends CommandBase {
  private OuterClimberSubsystem outerClimberSubsystem;
  private PIDController outerHorizontal;
  private double oldSpeed = 0.0;

  /** Creates a new OuterClimberHorizontalAlignmentCommand. */
  public OuterClimberHorizontalAlignmentCommand(OuterClimberSubsystem outerClimberSubsystem) {
    this.outerClimberSubsystem = new OuterClimberSubsystem();
    this.outerHorizontal = new PIDController(0.07, 0, 0);
    this.outerHorizontal.setSetpoint(0.0);
    this.outerHorizontal.setTolerance(1);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.outerClimberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = MathUtil.clamp(this.outerHorizontal.calculate(DriveSubsystem.getNavxRoll(), 0.0), -0.5, 0.5);
    double speedChange = (0.1 * speed + 0.9 * oldSpeed);
    this.outerClimberSubsystem.setSpeed(new double[] { -speedChange, speedChange });
    // SmartDashboard.putBoolean("OUTER CLIMBER HORIZONTAL", (DriveSubsystem.getNavxRoll() > 5));
    oldSpeed = speedChange;

    // SmartDashboard.putNumber("OuterKI", speed);
    // SmartDashboard.putNumber("MKC_OUTER", DriveSubsystem.getNavxRoll());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.outerHorizontal.close();
    this.outerClimberSubsystem.setSpeed(new double[] { 0.0, 0.0 });
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(DriveSubsystem.getNavxRoll()) < 2;
  }
}
