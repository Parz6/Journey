package Journey.output;

import Journey.input.X360Controller;

public class CarController {
    private ControlsOutput cO;
    private X360Controller x360;

    public CarController(ControlsOutput cO, boolean allowControllerOverride) {
        setControls(cO);
        if(allowControllerOverride) {
            this.x360 = new X360Controller();
            this.x360.start();
        }
    }
    public CarController(boolean allowControllerOverride) {
        this(new ControlsOutput(), allowControllerOverride);
    }
    public ControlsOutput getControls() {
        if(isHumanControlled())
            return x360.getControls();
        return cO;
    }
    public void setControls(ControlsOutput cO) {
        this.cO = cO;
    }
    public void setControls() {
        this.cO.clear();
    }
    public boolean isHumanControlled() {
        return x360 != null && x360.isEnabled();
    }
}
