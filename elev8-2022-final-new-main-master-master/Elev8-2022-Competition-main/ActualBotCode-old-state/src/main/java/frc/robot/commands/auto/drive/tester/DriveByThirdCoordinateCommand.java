// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.drive.tester;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;

public class DriveByThirdCoordinateCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  public double x, y;
  private double start_time;
  double lasttimestamp = 0;

  /** Creates a new DriveByThirdCoordinateCommand. */
  public DriveByThirdCoordinateCommand(DriveSubsystem driveSubsystem, double x, double y) {
    this.driveSubsystem = driveSubsystem;
    this.x = x;
    this.y = y;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    start_time = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lasttimestamp = Timer.getFPGATimestamp() - start_time;
    this.driveSubsystem.setSpeeds(this.driveSubsystem.speedcontrolforalign(x, y));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.driveSubsystem.setSpeeds(new double[] { 0, 0 });
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return (Math.abs(this.driveSubsystem.getDTheta()) < 0.5);
    return lasttimestamp > 1.5;
  }
}
