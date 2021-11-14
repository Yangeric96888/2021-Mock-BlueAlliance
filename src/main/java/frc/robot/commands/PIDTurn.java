// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.Timer;

public class PIDTurn extends CommandBase {
  private double angle;
  private int direction;
  private DriveTrain _driveTrain;
  private double speed;
  private double kP = 1, kI = 1, kD = 1;
  private double P, I, D;
  private double error;
  private Timer timer;
  private double lastError, lastTime;

  public PIDTurn(DriveTrain inputDriveTrain, double inputAngle) {
    _driveTrain = inputDriveTrain;
    angle = inputAngle;
    if (angle > 0) {
      direction = 1;
    } else {
      direction = -1;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _driveTrain.resetNavX();
    timer.start();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    error = angle - _driveTrain.getNAVXAngle();
    P = error;
    I += (timer.get() - lastTime) * error;
    D = (error - lastError) / (timer.get() - lastTime);
    lastError = error;
    lastTime = timer.get();

    speed = P * kP + I * kI + D * kD;
    _driveTrain.tankdrive(speed * direction, -speed * direction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (angle >= _driveTrain.getNavXAngle()) {
      _driveTrain.tankdrive(0, 0);
    }
    return false;
  }
}
