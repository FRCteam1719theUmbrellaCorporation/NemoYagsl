package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.CoralIntakeSubsystem;
// import frc.robot.Constants;

public class CoralIntakeWheelsCommand extends Command {
    CoralIntakeSubsystem m_CoralIntakeSubsystem;

    public CoralIntakeWheelsCommand(CoralIntakeSubsystem input) {
        m_CoralIntakeSubsystem = input;
    }
    public InstantCommand turnMotor(double speed) {
        return new InstantCommand(()-> 
            m_CoralIntakeSubsystem.intakeSpinWheels(speed)
        );
    }

    public InstantCommand stopMotors(){
        return new InstantCommand(()->
            m_CoralIntakeSubsystem.turnOffIntakeWheel()
        );
    }
}
