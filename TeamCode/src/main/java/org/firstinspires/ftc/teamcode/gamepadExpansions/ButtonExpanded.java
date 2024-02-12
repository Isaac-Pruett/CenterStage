package org.firstinspires.ftc.teamcode.gamepadExpansions;

public class ButtonExpanded {
    private boolean prevValue = false;
    private boolean toggled  = false;

    public void update(boolean newValue){
        if (newValue && isChanged(newValue)) {
            toggled = !toggled;
        }

        prevValue = newValue;
    }

    public boolean isChanged(boolean buttonValue){
        return buttonValue != prevValue;
    }
    public boolean isToggled(){
        return toggled;
    }
}
