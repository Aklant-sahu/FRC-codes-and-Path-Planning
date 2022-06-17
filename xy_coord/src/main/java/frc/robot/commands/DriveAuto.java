// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.Math;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveAuto extends CommandBase {
  /** Creates a new DriveAuto. */
  DriveTrain drivetrain;
  int counterx=0;
  int countery=0;
  public DriveAuto(DriveTrain subsystem) {
    drivetrain=subsystem;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counterx=0;
    countery=0;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.updatePose();
    drivetrain.speedcontrol(drivetrain.getNextX(counterx), drivetrain.getNextY(countery));
    if(Math.abs(drivetrain.getCurrX()-drivetrain.getNextX(counterx))<0.1 && Math.abs(drivetrain.getCurrX()-drivetrain.getNextX(counterx))<0.1 )
    {
      counterx++;
      countery++;
    }
    
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
