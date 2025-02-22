package frc.robot.commands.Intake;

import frc.robot.subsystems.intake.AlgaeIntakeSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AlgaeIntakeWheelsCommand {
    AlgaeIntakeSubsystem mAlgaeIntakeSubsystem;

    public InstantCommand turnMotor(int speed) {
        return new InstantCommand(()-> 
            mAlgaeIntakeSubsystem.turnIntakeWheels(speed)
        );
    }

    public InstantCommand stopMotors(){
        return new InstantCommand(()->
        mAlgaeIntakeSubsystem.stopIntakeMotors()
        );
    }
}
