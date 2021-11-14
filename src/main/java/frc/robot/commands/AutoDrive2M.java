// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoDrive2M extends CommandBase {
  /** Creates a new AutoDrive2M. */
  private DriveTrain driveTrain;
  private int distance;
  private double speed;
  private double error;
  private double kP = 0.5;
  
  private double SetPoint = 2 * 3.28084;

  public AutoDrive2M(DriveTrain dt, int distance, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.distance = distance;
    SetPoint = distance * 3.28084;
    this.speed = speed;
    driveTrain = dt;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Get the current Position of the DriveTrain
    double sensorPosition = driveTrain.getPosition();
    
    error = SetPoint - sensorPosition;

    double outputSpeed = error * kP;
    
    driveTrain.tankDrive(outputSpeed, outputSpeed);
  
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
