package frc.robot.commands.Controls;

import java.util.function.DoubleSupplier;
import frc.robot.RobotContainer;
import utils.Reef.Location;

// reef selector positions
public class SelectReef {
    private DoubleSupplier x;
    private DoubleSupplier y;

    // constructor
    public SelectReef(DoubleSupplier x, DoubleSupplier y) {
        this.x = x;
        this.y = y;
    }

    // 
    public void execute() {
        double x = this.x.getAsDouble();
        double y = this.y.getAsDouble();

        // early return
        if (Math.sqrt(x * x + y * y) < 0.7) return;

        double angle = Math.toDegrees(Math.atan2(y, x));

        if (angle >= 150) {
            RobotContainer.loc = Location.J;
        } else if (angle >=  120) {
            RobotContainer.loc = Location.I;
        } else if (angle >= 90) {
            RobotContainer.loc = Location.H;
        } else if (angle >= 60) {
            RobotContainer.loc = Location.G;
        } else if (angle >= 30) {
            RobotContainer.loc = Location.F;
        } else if (angle >= 0) {
            RobotContainer.loc = Location.E;
        } else if (angle >= -30) {
            RobotContainer.loc = Location.D;
        } else if (angle >= -60) {
            RobotContainer.loc = Location.C;
        } else if (angle >= -90) {
            RobotContainer.loc = Location.B;
        } else if (angle >= -120) {
            RobotContainer.loc = Location.A;
        } else if (angle >= -150) {
            RobotContainer.loc = Location.L;
        } else if (angle >= -180) {
            RobotContainer.loc = Location.K;
        }
    }
}
