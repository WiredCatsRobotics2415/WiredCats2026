package frc.utils.tuning;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;

public class Characterizer {
    public static final TuneableBoolean enableSafety = new TuneableBoolean(true, "EnableCharSafties");

    public ArrayList<Command> commands = new ArrayList<Command>();
}