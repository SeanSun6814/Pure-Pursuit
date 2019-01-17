package frc.robot.commands;

import Obj.MessageLevel;
import edu.wpi.first.wpilibj.command.Command;
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

public abstract class CommandBase extends Command {
    private String systemName = "Default";
    private Logger logger = Logger.getInstance();

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
        logger.log(message, systemName, messageLevel);
    }

    protected void log(String title, Object message) {
        logger.log(message, title, MessageLevel.Default);
    }
}
