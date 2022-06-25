// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveAuto extends CommandBase {
  /** Creates a new DriveAuto. */
  private final DriveTrain driveTrain;
  public boolean todo;
  public double setp=0;
  public float angle;
  boolean ret;
  Timer timer;
  double x,y;
  /** Creates a new DriveWithJoysticks. */
  public DriveAuto(DriveTrain dt,boolean tod,boolean ret1,double x1,double y1) {
    driveTrain=dt;
    addRequirements(driveTrain);
    todo=tod;
    x=x1;
    y=y1;
    ret=ret1;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ret=false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(todo==true){
    
      double[] speed=driveTrain.pidRotate(x,y);
      driveTrain.driveWith(speed[0], speed[1]);
      ret=driveTrain.checkAng();
      
     
    
     
    }
    else if(todo==false){
     
      double[] speed=driveTrain.pidForward(x,y);
      driveTrain.driveWith(speed[0], speed[1]);
      ret=driveTrain.checkDist();
      
      System.out.println("moving robot straight");
      // }
      // ret=true;
    }
    // driveTrain.res();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ret;
  }
}
