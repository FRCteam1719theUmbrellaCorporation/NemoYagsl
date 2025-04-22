// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.outake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Elevator.ElevatorSubsytem.HeightLevels;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeCoralEndeffector extends SequentialCommandGroup {

  final private static SequentialCommandGroup emptyCommand = new SequentialCommandGroup(Commands.none());
  /**
   * This command tells the arm to move down from a spot hovering above the position to grab coral
   * 
   * @param cmd: endeffector PID move command
   * @return Sequential co
   */
  public static SequentialCommandGroup quickIntake(EndEffectorPIDCommand cmd) {
    // doesnt move if it isnt in the right position
    if (!cmd.comparePosition(HeightLevels.INTAKE_WITH_ARN_DOWN).getAsBoolean()) return emptyCommand;

    return new SequentialCommandGroup(
      cmd.moveBoth(HeightLevels.INTAKE),
      new WaitUntilCommand(cmd.isAtPos())
    );
  }

  /**
   * Moves the arm from above the intake positon and then back up to hovering above
   * 
   * @param cmd: endeffector PID move command
   * @return Sequential command
   */
  public static SequentialCommandGroup quickIntakeFacingDown(EndEffectorPIDCommand cmd) {
    return new SequentialCommandGroup(
      cmd.moveBoth(HeightLevels.INTAKE),
      new WaitUntilCommand(cmd.isAtPos()),
      cmd.moveBoth(HeightLevels.INTAKE_WITH_ARN_DOWN)
    );
  }

  /**
   * Moves the arm from above the intake positon, picks up a coral, 
   * and flips the then proceeds to move the arm back to it's zero position
   * 
   * @param cmd: endeffector PID move command
   * @return Sequential command
   */
  public static SequentialCommandGroup quickIntakeToUp(EndEffectorPIDCommand cmd) {
    return new SequentialCommandGroup(
      cmd.moveBoth(HeightLevels.INTAKE),
      new WaitUntilCommand(cmd.isAtPos()),
      cmd.moveBoth(HeightLevels.INTAKE_PRE_DOWN),
      new WaitUntilCommand(cmd.isAtPos()),
      cmd.moveBoth(HeightLevels.ZERO, false),
      new WaitUntilCommand(cmd.isAtPos())
    );
  }

  /**
   * Intakes from a low positon, to then move the elevator to its 0 pos. may help with driving
   * 
   * 
   * @param cmd: endeffector PID move command
   * @return Sequential command
   */
  // public static SequentialCommandGroup IntakeDownToZeroPosition(EndEffectorPIDCommand cmd) {
  //   // doesnt move if it isnt in the right position
  //   if (!cmd.comparePosition(HeightLevels.INTAKE_WITH_ARN_DOWN).getAsBoolean()) return emptyCommand;

  //   return new SequentialCommandGroup(
  //     IntakeCoralEndeffector.quickIntake(cmd),
  //     cmd.moveBoth(HeightLevels.INTAKE_UP),
  //       new WaitUntilCommand(cmd.isAtPos()),
  //       cmd.moveBoth(HeightLevels.ZERO),
  //       new WaitUntilCommand(cmd.isAtPos())
  //   );
  // }

  // old intake.
  public static SequentialCommandGroup intakeToUp(EndEffectorPIDCommand cmd) {
    return new SequentialCommandGroup(
        cmd.moveBoth(HeightLevels.INTAKE_UP),
        new WaitUntilCommand(cmd.isAtPos()),
        cmd.moveBoth(HeightLevels.INTAKE_PRE_DOWN),
        new WaitUntilCommand(cmd.isAtPos()),
        cmd.moveBoth(HeightLevels.INTAKE),
        new WaitUntilCommand(cmd.isAtPos()).withTimeout(8), // will go back up :)
        cmd.moveBoth(HeightLevels.INTAKE_UP),
        new WaitUntilCommand(cmd.isAtPos()),
        cmd.moveBoth(HeightLevels.ZERO),
        new WaitUntilCommand(cmd.isAtPos())
      );
  }
}
