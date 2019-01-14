package Obj;

import frc.robot.Logger;

/**
 * Summary: This class is designed to make logging easier in systems. It avoids
 * redundantly having to write Robot.logger.log(message, sender,
 * MessageLevel.Info); over and over again. Instead, it simplifies it to just
 * log(msg);
 * 
 * To use: inherit this class and set the system name for it to autofill the
 * sender param
 */

public abstract class LogBase {
    private String systemName = "Default";

    /**
     * @param systemName: set the sender autofill value for all logs
     */
    protected void setLogSenderName(String systemName) {
        this.systemName = systemName;
    }

    protected void log(Object message) {
        log(message, MessageLevel.Info);
    }

    protected void log(Object message, MessageLevel messageLevel) {
        Logger.getInstance().log(message, systemName, messageLevel);
    }
}
