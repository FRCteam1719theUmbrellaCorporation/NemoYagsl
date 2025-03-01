// /*
//  * Moves elevator to setpoint
//  * 
//  */

// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Elevator;

// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.RelativeEncoder;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.subsystems.Elevator.ElevatorSubsytem;
// import frc.robot.subsystems.intake.AlgaeIntakeSubsystem;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class ElevatorPIDMoveCommand extends Command {
//   /** Creates a new ElevatorPIDMove. */

//     private final ElevatorSubsytem intake;
//     private final PIDController m_ElevatorHeight;
//     public double SETHEIGHT;
//     private final RelativeEncoder m_HEIGHT_ENCODER;
  
  
//     public ElevatorPIDMoveCommand(ElevatorSubsytem intake, PIDController m_ElevatorHeight, RelativeEncoder m_HEIGHT_ENCODER) {
//       addRequirements(intake);
//       this.intake = intake;
//       this.m_ElevatorHeight = m_ElevatorHeight;
//       this.m_HEIGHT_ENCODER = m_HEIGHT_ENCODER;
//     }
  
//     // Called when the command is initially scheduled.
//     @Override
//     public void initialize() {
//       m_HEIGHT_ENCODER.setPosition(0);
//     }
  
//     // Called every time the scheduler runs while the command is scheduled.
//     @Override
//     public void execute() {}
  
//     public void setHeight(double height) {
//       intake.setGoal(height);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
