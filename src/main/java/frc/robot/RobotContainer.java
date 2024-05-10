// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SetMotorPositionBangBang;
import frc.robot.commands.SetMotorPositionProportional;
import frc.robot.commands.SimpleSetSpeedComand;
import frc.robot.subsystems.Intake;

public class RobotContainer {
  CommandXboxController controller;
  Intake intake;

  public RobotContainer() {
    controller = new CommandXboxController(0);
    intake = new Intake();
    configureBindings();
  }

  private void configureBindings() {

    // set the motor to predefined speed when "A" button on the controller is
    // pressed
    controller.povUp().whileTrue(new SimpleSetSpeedComand(
        intake,
        Constants.SIMPLE_MOTOR_SPEED));

    controller.povDown().whileTrue(new SimpleSetSpeedComand(
        intake,
        -Constants.SIMPLE_MOTOR_SPEED));

    controller.b().whileTrue(new SetMotorPositionBangBang(
        intake,
        90));

    controller.y().whileTrue(new SetMotorPositionProportional(
        intake,
        90));

  }

  public void zeroOdom() {
    this.intake.zeroOdom();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
