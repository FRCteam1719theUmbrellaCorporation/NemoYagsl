// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.outake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Elevator.ElevatorSubsytem.HeightLevels;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceCoralCommand extends SequentialCommandGroup {
  public static SequentialCommandGroup placeAt(EndEffectorPIDCommand cmd, HeightLevels height) {
    return new SequentialCommandGroup(
        cmd.moveBoth(HeightLevels.INTAKE_UP),
        new WaitUntilCommand(cmd.isAtPos()),
        cmd.moveBoth(height),
        new WaitUntilCommand(cmd.isAtPos()),
        cmd.moveBoth(HeightLevels.INTAKE_UP),
        new WaitUntilCommand(cmd.isAtPos()),
        cmd.moveBoth(HeightLevels.ZERO)
        // new WaitUntilCommand(cmd.isAtPos()),
      );
  } 
}
