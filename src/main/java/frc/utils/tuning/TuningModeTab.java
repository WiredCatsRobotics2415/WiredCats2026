package frc.utils.tuning;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.constants.RuntimeConstants;
import java.time.LocalDate;
import java.time.LocalTime;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TuningModeTab {
    private static TuningModeTab instance;
    private ShuffleboardTab thisTab;

    private int currentLogRun = 1;

    private TuningModeTab() {
        thisTab = Shuffleboard.getTab("Tuning");

        addCommand("Stop Signal Logger", new InstantCommand(() -> {
            System.out.println("Saving log as log #" + currentLogRun);
            SignalLogger.stop();
        }).ignoringDisable(true));
        addCommand("Start Signal Logger", new InstantCommand(() -> {
            currentLogRun += 1;
            System.out.println("Starting signal logger with run #" + currentLogRun);
            LocalDate date = LocalDate.now();
            LocalTime time = LocalTime.now();
            SignalLogger.start();
        }).ignoringDisable(true));
    }

    /**
     * Adds a characterizer to the tab. In each characterizer class, run this method LAST in the constructor
     */
    public void addCharacterizer(String name, Characterizer characterizer) {
        ShuffleboardLayout layout = thisTab.getLayout(name, BuiltInLayouts.kList);

        for (Command command : characterizer.commands) {
            layout.add(command);
        }
    }

    public static void enableTuningMode() {
        if (instance == null) {
            instance = new TuningModeTab();
        }
    }

    public static TuningModeTab getInstance() {
        if (instance == null && RuntimeConstants.TuningMode) instance = new TuningModeTab();
        return instance;
    }

    public void addBoolSupplier(String title, BooleanSupplier supplier) {
        thisTab.addBoolean(title, supplier).withWidget("Boolean Box");
    }

    public void addDoubleSupplier(String title, DoubleSupplier supplier) {
        thisTab.addDouble(title, supplier).withWidget("Number");
    }

    public void addCommand(String title, Command command) {
        thisTab.add(title, command).withWidget("Command");
    }
}