// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.climber.inner;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.inner.InnerClimberSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;

public class InnerClimberHorizontalAlignmentCommand extends CommandBase {
  private InnerClimberSubsystem innerClimberSubsystem;
  private PIDController innerHorizontal;
  double oldSpeed = 0.0;

  /** Creates a new InnerClimberHorizontalAlignmentCommand. */
  public InnerClimberHorizontalAlignmentCommand(InnerClimberSubsystem innerClimberSubsystem) {
    this.innerClimberSubsystem = innerClimberSubsystem;
    this.innerHorizontal = new PIDController(0.035, 0, 0);
    this.innerHorizontal.setSetpoint(0.0);
    this.innerHorizontal.setTolerance(2);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.innerClimberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = MathUtil.clamp(this.innerHorizontal.calculate(DriveSubsystem.getNavxRoll(), 0.0), -0.8, 0.8);
    double speedChange = (0.6 * speed + 0.4 * oldSpeed);
    this.innerClimberSubsystem.setSpeed(new double[] { -speedChange, speedChange });
    // SmartDashboard.putBoolean("INNER CLIMBER HORIZONTAL", (DriveSubsystem.getNavxRoll() > 5));
    oldSpeed = speedChange;

    // SmartDashboard.putNumber("InnerKI", speed);
    // SmartDashboard.putNumber("MKC_INNER", DriveSubsystem.getNavxRoll());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.innerHorizontal.close();
    this.innerClimberSubsystem.setSpeed(new double[] { 0.0, 0.0 });
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(DriveSubsystem.getNavxRoll()) < 2;
    // return false;
  }
}
