// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.intakestarter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.IntakeStarterSubsystem;

public class IntakeStartCommand extends CommandBase {
  private IntakeStarterSubsystem intakeStarterSubsystem;
  private Supplier<Boolean> upside, downside;

  /** Creates a new IntakeStartCommand. */
  public IntakeStartCommand(IntakeStarterSubsystem intakeStarterSubsystem, Supplier<Boolean> upside,
      Supplier<Boolean> downside) {
    this.intakeStarterSubsystem = intakeStarterSubsystem;
    this.upside = upside;
    this.downside = downside;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intakeStarterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (this.upside.get()) {
      this.intakeStarterSubsystem.setSpeed(0.7);
    } else if (this.downside.get()) {
      this.intakeStarterSubsystem.setSpeed(-0.7);
    } else {
      this.intakeStarterSubsystem.setSpeed(0.1);
    }
    SmartDashboard.putBoolean("UPO", this.upside.get());
    SmartDashboard.putBoolean("DOWNO", this.downside.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.intakeStarterSubsystem.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
