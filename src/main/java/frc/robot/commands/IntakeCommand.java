// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ClawPosition;


public class IntakeCommand extends CommandBase {

  private Arm arm;
  private ClawPosition position;


  /** Creates a new Claw. */
  public IntakeCommand(Arm arm,ClawPosition object) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    position = object;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setClawPosition(position);
    arm.runRollers(Constants.Arm.ROLLER_IN);
  
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.runRollers(Constants.Arm.ROLLER_OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
