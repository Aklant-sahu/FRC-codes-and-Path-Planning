// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class Driveforward extends CommandBase {
  /** Creates a new Driveforward. */
  DriveSubsystem m_DriveSubsystem;
  private boolean finish=false;
  Timer timer;
  public Driveforward( DriveSubsystem Subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DriveSubsystem=Subsystem;
    addRequirements(m_DriveSubsystem);
    timer=new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    while(timer.get()<5){
      m_DriveSubsystem.driveauto();
    }
    finish=true;


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
