package frc.visualization;

import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.subsystems.shooter.Shooter;
import frc.subsystems.turret.Turret;

public class RobotParts {
    private final Shooter shooter = Shooter.getInstance();
    private final Turret turret = Turret.getInstance();
    private final CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
}
