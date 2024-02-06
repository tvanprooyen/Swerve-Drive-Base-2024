package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class CommandXboxControllerExpand extends CommandXboxController {

    public CommandXboxControllerExpand(int port) {
        super(port);
    }
    
    /**
     * Get the POV value of the controller
     * @return  The POV value of the controller
     */
    public int getPOV() {
        if(povCenter().getAsBoolean()) {
            return -1;
        } else if(povUp().getAsBoolean()) {
            return 0;
        } else if(povUpRight().getAsBoolean()) {
            return 45;
        } else if(povRight().getAsBoolean()) {
            return 90;
        } else if(povDownRight().getAsBoolean()) {
            return 135;
        } else if(povDown().getAsBoolean()) {
            return 180;
        } else if(povDownLeft().getAsBoolean()) {
            return 225;
        } else if(povLeft().getAsBoolean()) {
            return 270;
        } else if(povUpLeft().getAsBoolean()) {
            return 315;
        } else {
            return -1;
        }
    }
}
