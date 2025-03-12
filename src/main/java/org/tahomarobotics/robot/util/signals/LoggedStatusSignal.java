/*
 * Copyright 2025 Tahoma Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

package org.tahomarobotics.robot.util.signals;

import com.ctre.phoenix6.BaseStatusSignal;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

public record LoggedStatusSignal(String name, BaseStatusSignal signal) {
    private static BaseStatusSignal[] getBaseSignals(LoggedStatusSignal[] signals) {
        return Arrays.stream(signals).map(LoggedStatusSignal::signal).toArray(BaseStatusSignal[]::new);
    }

    public static void setUpdateFrequencyForAll(LoggedStatusSignal[] signals, double updateFrequency) {
        BaseStatusSignal.setUpdateFrequencyForAll(updateFrequency, getBaseSignals(signals));
    }

    public static void refreshAll(LoggedStatusSignal[] signals) {
        BaseStatusSignal.refreshAll(getBaseSignals(signals));
    }

    public static void waitForAll(double timeout, LoggedStatusSignal[] signals) {
        BaseStatusSignal.waitForAll(timeout, getBaseSignals(signals));
    }

    public static void log(String prefix, LoggedStatusSignal[] signals) {
        prefix += "Status Signals/";

        for (var signal : signals) {
            String prefix_ = prefix + "/" + signal.name + "/";

            Logger.recordOutput(prefix_ + "value", signal.signal().getValueAsDouble());
            Logger.recordOutput(prefix_ + "status", signal.signal().getStatus());
            Logger.recordOutput(prefix_ + "units", signal.signal().getUnits());
            Logger.recordOutput(prefix_ + "latency", signal.signal().getTimestamp().getLatency());
        }
    }
}
