package frc.robot.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class CommandXboxControllerExpand extends CommandXboxController {

    private final XboxController m_hid;

    public CommandXboxControllerExpand(int port) {
        super(port);
        m_hid = getHID();
    }
    
    /**
     * Get the POV value of the controller
     * @return  The POV value of the controller
     */
    public int getPOV() {
        return m_hid.getPOV();
    }
}
