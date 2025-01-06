package org.tahomarobotics.robot.util.loggers;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import org.tahomarobotics.robot.util.CalibrationData;

@SuppressWarnings("rawtypes")
@CustomLoggerFor(CalibrationData.class)
public class CalibrationDataLogger extends ClassSpecificLogger<CalibrationData> {
    public CalibrationDataLogger() {
        super(CalibrationData.class);
    }

    @Override
    protected void update(EpilogueBackend backend, CalibrationData data) {
        backend.log("data", data.toString());
    }
}
