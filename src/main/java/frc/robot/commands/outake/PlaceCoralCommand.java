// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.outake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator.ElevatorSubsytem.HeightLevels;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import utils.Reef.Level; // i will end it

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceCoralCommand extends SequentialCommandGroup {
  private static EndEffectorPIDCommand m_cmd;
  private static SwerveSubsystem m_swerve;
  public static volatile HeightLevels height1;
  public static volatile HeightLevels height2;

  public PlaceCoralCommand(EndEffectorPIDCommand cmd, SwerveSubsystem swerve) {
    m_cmd = cmd;
    m_swerve = swerve;
    System.out.println(Robot.reefLevel);
  }

  // public static SequentialCommandGroup placeAt(Level rLevel) {
  
  //   // System.out.println(RobotContainer.level);

  //   if (height1 == null) {
  //     PlaceCoralCommand.height1 = HeightLevels.LOW_PRE;
  //     PlaceCoralCommand.height2 = HeightLevels.LOW;
  //   }
    
  //   // switch(rLevel) {
  //   //   case L2:
  //   //     PlaceCoralCommand.height1 = HeightLevels.LOW_PRE;
  //   //     PlaceCoralCommand.height2 = HeightLevels.LOW;

  //   //     break;
  //   //   case L3:
  //   //     PlaceCoralCommand.height1 = HeightLevels.Middle_PRE;
  //   //     PlaceCoralCommand.height2 = HeightLevels.MIDDLE;
  //   //     break;
  //   //   default:
  //   //     PlaceCoralCommand.height1 = HeightLevels.HIGH_PRE;
  //   //     PlaceCoralCommand.height2 = HeightLevels.HIGH;
  //   //     break;
  //   // }

  //   return new SequentialCommandGroup(
  //       m_cmd.moveBoth(PlaceCoralCommand.height1),
  //       new WaitUntilCommand(m_cmd.isAtPos()),
  //       m_cmd.moveBoth(PlaceCoralCommand.height2),
  //       new WaitUntilCommand(m_cmd.isAtPos()),
  //       new WaitCommand(2),

  //       m_cmd.moveBoth(HeightLevels.ZERO),
  //       new WaitUntilCommand(m_cmd.isAtPos())
        
  //     );
  // } 

  // i'd rather kms then write this code again

  public static SequentialCommandGroup placeL2() {
    return new SequentialCommandGroup(
        m_cmd.moveBoth(HeightLevels.LOW_PRE),
        new WaitUntilCommand(m_cmd.isAtPos()),
        m_cmd.moveBoth(HeightLevels.LOW),
        new WaitUntilCommand(m_cmd.isAtPos()),
        new WaitCommand(2),
        m_cmd.moveBoth(HeightLevels.ZERO),
        new WaitUntilCommand(m_cmd.isAtPos())
        
      );
  }

  public static SequentialCommandGroup placeL3() {
    return new SequentialCommandGroup(
        m_cmd.moveBoth(HeightLevels.Middle_PRE),
        new WaitUntilCommand(m_cmd.isAtPos()),
        m_cmd.moveBoth(HeightLevels.MIDDLE),
        new WaitUntilCommand(m_cmd.isAtPos()),
        new WaitCommand(2),
        m_cmd.moveBoth(HeightLevels.ZERO),
        new WaitUntilCommand(m_cmd.isAtPos())
      );
  }

  public static SequentialCommandGroup placeL4() {
    return new SequentialCommandGroup(
        m_cmd.moveBoth(HeightLevels.HIGH_PRE),
        new WaitUntilCommand(m_cmd.isAtPos()),
        m_cmd.moveBoth(HeightLevels.HIGH),
        new WaitUntilCommand(m_cmd.isAtPos()),
        new WaitCommand(2),
        m_cmd.moveBoth(HeightLevels.ZERO),
        new WaitUntilCommand(m_cmd.isAtPos())
      );
  }

}
