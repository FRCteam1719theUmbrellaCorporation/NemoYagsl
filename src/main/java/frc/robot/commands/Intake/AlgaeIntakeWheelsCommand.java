// package frc.robot.commands.Intake;

// import frc.robot.subsystems.intake.AlgaeIntakeSubsystem;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.Command;

// public class AlgaeIntakeWheelsCommand extends Command{
//     AlgaeIntakeSubsystem mAlgaeIntakeSubsystem;

//     public AlgaeIntakeWheelsCommand(AlgaeIntakeSubsystem input) {
//         mAlgaeIntakeSubsystem = input;
//     } 

//     public InstantCommand turnMotor(int speed) {
//         return new InstantCommand(()-> 
//             mAlgaeIntakeSubsystem.turnIntakeWheels(speed)
//         );
//     }

//     public InstantCommand stopMotors(){
//         return new InstantCommand(()->
//         mAlgaeIntakeSubsystem.stopIntakeMotors()
//         );
//     }
// }
