package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.CoralIntakeSubsystem;

public class CoralIntakeWheelsCommand extends Command {
    CoralIntakeSubsystem mCoralIntakeSubsystem;

    public CoralIntakeWheelsCommand(CoralIntakeSubsystem input) {
        mCoralIntakeSubsystem = input;
    }
    public InstantCommand turnMotor(int speed) {
        return new InstantCommand(()-> 
            mCoralIntakeSubsystem.intakeSpinWheels(speed)
        );
    }

    public InstantCommand stopMotors(){
        return new InstantCommand(()->
        mCoralIntakeSubsystem.turnOffIntakeWheel()
        );
    }
}
