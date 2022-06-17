// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.drive.tester;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class DriveByAFourthCoordinateCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private double x, y, distance_from_goal;
  private double start_time;
  double lasttimestamp = 0;

  /** Creates a new DriveByAFourthCoordinateCommand. */
  public DriveByAFourthCoordinateCommand(DriveSubsystem driveSubsystem, double x, double y, double distance_from_goal) {
    this.driveSubsystem = driveSubsystem;
    this.x = x;
    this.y = y;
    this.distance_from_goal = distance_from_goal;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.start_time = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lasttimestamp = Timer.getFPGATimestamp() - start_time;
    this.driveSubsystem
        .setSpeeds(this.driveSubsystem.speedcontrolforalign_and_distance_correction(x, y, distance_from_goal));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.driveSubsystem.setSpeeds(new double[] { 0, 0 });
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;
    return lasttimestamp > 1.5;
  }
}
