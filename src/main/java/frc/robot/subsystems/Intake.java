// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  SparkMax intakeMotor;

  /** Creates a new Intake. */
  public Intake() {
    // instantiate the intake motor
    intakeMotor = new SparkMax(Constants.DEVICE_ID, MotorType.kBrushless);

    // configure the intake motor
    SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    intakeMotorConfig.idleMode(IdleMode.kCoast);

    // apply the config to the motor
    intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public SparkMax getMotor() {
    return intakeMotor;
  }

  public RelativeEncoder getMotorEncoder() {
    return intakeMotor.getEncoder();
  }

  public double getEncoderPosition() {
    return intakeMotor.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("MotorEncoderVal", getEncoderPosition());
  }

  public void zeroOdom() {
    getMotorEncoder().setPosition(0);
  }

  public void setSpinSpeed(double speed) {
    intakeMotor.set(speed);
  }
}
