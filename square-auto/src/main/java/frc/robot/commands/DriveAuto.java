// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveAuto extends CommandBase {
  /** Creates a new DriveAuto. */
  private final DriveTrain driveTrain;
  public boolean todo;
  public double setp;
  public float angle;
  /** Creates a new DriveWithJoysticks. */
  public DriveAuto(DriveTrain dt,boolean tod,double set,float angl) {
    driveTrain=dt;
    addRequirements(driveTrain);
    todo=tod;
    setp=set;
    angle=angl;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(todo==true){
      driveTrain.orient(angle);
    }
    else if(todo==false){
      double left=driveTrain.leftencpos();
      double right=driveTrain.rightencpos();
      driveTrain.drive1m(left, right,setp);
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
