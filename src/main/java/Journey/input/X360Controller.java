package Journey.input;

import Journey.output.ControlsOutput;
import com.studiohartman.jamepad.ControllerAxis;
import com.studiohartman.jamepad.ControllerButton;
import com.studiohartman.jamepad.ControllerIndex;
import com.studiohartman.jamepad.ControllerManager;
import com.studiohartman.jamepad.ControllerUnpluggedException;

// Credits to r0bbie; this class was adapted from src/main/java/wildfire/wildfire/obj/Human.java at https://github.com/robbai/Wildfire

public class X360Controller extends Thread {
    private static final int CONTROLLER_INDEX = 0; // For ControllerManager.getControllerIndex(int index)

    private boolean enabled;
    private ControlsOutput cO;
    private ControllerManager controllers;

    public X360Controller() {
        this.enabled = false;
        this.cO = new ControlsOutput();
        this.controllers = new ControllerManager();
    }

    public void run(){
        controllers.initSDLGamepad();
        ControllerIndex currController = controllers.getControllerIndex(CONTROLLER_INDEX);

        while(!this.isInterrupted()){
            controllers.update();
            try{
                //Toggle enabled flag
                if(currController.isButtonJustPressed(ControllerButton.RIGHTSTICK))
                    this.setEnabled(!this.isEnabled());

                if(!this.isEnabled())
                    continue;

                cO.withJump(currController.getAxisState(ControllerAxis.TRIGGERRIGHT) > 0.1177); // Imitate in-game
                cO.withBoost(currController.isButtonPressed(ControllerButton.A));
                cO.withSlide(currController.isButtonPressed(ControllerButton.X));

                float brake = currController.isButtonPressed(ControllerButton.LEFTBUMPER) ? 1.0F : 0F;
                cO.withThrottle(currController.getAxisState(ControllerAxis.TRIGGERLEFT) - brake);

                float pitch = -currController.getAxisState(ControllerAxis.LEFTY);
                float yaw = currController.getAxisState(ControllerAxis.LEFTX);

                // Deadzones, maybe done wrong
                if(Math.abs(pitch) < 0.27)
                    pitch = 0;
                if(Math.abs(yaw) < 0.27)
                    yaw = 0;

                cO.withPitch(pitch);
                cO.withYaw(yaw);
                cO.withSteer(yaw);
                cO.withRoll(currController.isButtonPressed(ControllerButton.X) ? yaw : 0);
            } catch(ControllerUnpluggedException e) {
                continue;
            }
        }
        controllers.quitSDLGamepad();
    }

    public ControlsOutput getControls(){
        return cO;
    }

    public boolean isEnabled(){
        return enabled;
    }

    public void setEnabled(boolean enabled){
        this.enabled = enabled;
    }
}
