// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.kauailabs.navx.frc.AHRS;

public class DriveTrain extends SubsystemBase {
  private final WPI_TalonSRX _leftDriveTalon;
  private final WPI_TalonSRX _rightDriveTalon;
  private AHRS navx = new AHRS(SPI.Port.kMXP);
  private double circumference  = 47.12; // in centimeters
  private final int ticksInOneRevolution = 4096; 
 
  private DifferentialDrive _diffDrive;
  
  // creates a new drive train 
  public DriveTrain() {
    _leftDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.LeftDriveTalonPort);
    _rightDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.RightDriveTalonPort);

    _leftDriveTalon.setInverted(false);
    _rightDriveTalon.setInverted(false);

    _diffDrive = new DifferentialDrive(_leftDriveTalon, _rightDriveTalon);

    _leftDriveTalon.configFactoryDefault();
    _leftDriveTalon.setInverted(false);
    _leftDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    _rightDriveTalon.configFactoryDefault();
    _rightDriveTalon.setInverted(false);
    _rightDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void resetEncoders(){
    _leftDriveTalon.setSelectedSensorPosition(0,0,10);
    _rightDriveTalon.setSelectedSensorPosition(0,0,10);
  }

    public void setInverted(){
    _rightDriveTalon.setSensorPhase(true);
  }

  public double getPosition(){
    return (((_leftDriveTalon.getSelectedSensorPosition(0) + _rightDriveTalon.getSelectedSensorPosition(0))/2) * (circumference/ticksInOneRevolution));
  }

  public double getVelocity(){
    return (((_leftDriveTalon.getSensorCollection().getPulseWidthVelocity() + _rightDriveTalon.getSensorCollection().getPulseWidthVelocity())/2) * (circumference/ticksInOneRevolution));
 }

 public double getAngleAndReset(){
   double degrees = navx.getAngle();
   navx.reset();
   return degrees;
 }

 public double getAngle(){
   return navx.getAngle(); 
 }

 public void reset(){
   navx.reset();
 }
 
 public void tankDrive(double leftSpeed, double rightSpeed) {
    _diffDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void arcadeDrive(double speed, double rotation) {
    _diffDrive.arcadeDrive(speed, rotation);
  }

}
