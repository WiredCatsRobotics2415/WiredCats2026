package frc.utils.tuning;

import frc.constants.RuntimeConstants;
import java.util.ArrayList;
import java.util.function.Consumer;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class TuneableBoolean {
    private static ArrayList<TuneableBoolean> all = new ArrayList<TuneableBoolean>();
    private static int indexToUpdate = 0;

    private LoggedNetworkBoolean thisNetworkBoolean;
    private boolean previousValue;
    private ArrayList<Consumer<Boolean>> listeners;

    public TuneableBoolean(boolean defaultValue, String key) {
        this.previousValue = defaultValue;
        if (RuntimeConstants.TuningMode) {
            thisNetworkBoolean = new LoggedNetworkBoolean("/Tuning/" + key);
            thisNetworkBoolean.setDefault(defaultValue);
            listeners = new ArrayList<Consumer<Boolean>>();
            all.add(this);
        }
    }

    public boolean get() {
        return previousValue;
    }

    /**
     * Consumer to call when value is changed from NT. Do NOT call get() on this tuneable, because the value will not have changed yet.
     *
     * @param toCall
     */
    public void addListener(Consumer<Boolean> toCall) {
        listeners.add(toCall);
    }

    private void updateFromNT() {
        boolean entry = thisNetworkBoolean.get();
        if (entry != previousValue) {
            previousValue = entry;
            for (Consumer<Boolean> r : listeners)
                r.accept(entry);
        }
    }

    public static void periodic() {
        all.get(indexToUpdate).updateFromNT();
        indexToUpdate += 1;
        if (indexToUpdate == all.size()) {
            indexToUpdate = 0;
        }
    }
}