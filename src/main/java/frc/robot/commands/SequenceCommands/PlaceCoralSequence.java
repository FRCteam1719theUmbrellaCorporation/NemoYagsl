// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.SequenceCommands;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.outake.EndEffectorPIDCommand;
// import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class PlaceCoralSequence extends SequentialCommandGroup {
//   private static EndEffectorPIDCommand m_EffectorPIDCommand;
//   private static SwerveSubsystem m_SwerveSubsystem;

//   /** Creates a new PlaceCoralSequence. */
//   public PlaceCoralSequence(EndEffectorPIDCommand endEffCmd, SwerveSubsystem swerveatron) {
//     m_EffectorPIDCommand = endEffCmd;
//     m_SwerveSubsystem = swerveatron;
//   }

//   // public static SequentialCommandGroup atPosition(HeightLevels height) {
//   //   return new SequentialCommandGroup(
//   //     new ParallelCommandGroup(m_EffectorPIDCommand.moveBoth(height)), // TODO ADD POSITION.
//   //     new WaitUntilCommand( /*we need something here to check pos*/) // compare to target pos
//   //     // we need to drive backwards now.
//   //   );
//   // }
// }
