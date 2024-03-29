package Journey.output;

import rlbot.ControllerState;

public class ControlsOutput implements ControllerState {

    // 0 is straight, -1 is hard left, 1 is hard right.
    private float steer;

    // -1 for front flip, 1 for back flip
    private float pitch;

    // 0 is straight, -1 is hard left, 1 is hard right.
    private float yaw;

    // 0 is straight, -1 is hard left, 1 is hard right.
    private float roll;

    // 0 is none, -1 is backwards, 1 is forwards
    private float throttle;

    private boolean jumpDepressed;
    private boolean boostDepressed;
    private boolean handbrakeDepressed;
    private boolean useItemDepressed;

    public ControlsOutput() {
        clear();
    }

    public ControlsOutput clear() {
        steer = pitch = yaw = roll = throttle = 0F;
        jumpDepressed = boostDepressed = handbrakeDepressed = useItemDepressed = false;
        return this;
    }

    public ControlsOutput withSteer(float steer) {
        this.steer = clamp(steer);
        return this;
    }
    public ControlsOutput withSteer(double steer) {
        return withSteer((float) steer);
    }

    public ControlsOutput withPitch(float pitch) {
        this.pitch = clamp(pitch);
        return this;
    }
    public ControlsOutput withPitch(double pitch) {
        return withPitch((float) pitch);
    }

    public ControlsOutput withYaw(float yaw) {
        this.yaw = clamp(yaw);
        return this;
    }
    public ControlsOutput withYaw(double yaw) {
        return withYaw((float) yaw);
    }

    public ControlsOutput withRoll(float roll) {
        this.roll = clamp(roll);
        return this;
    }
    public ControlsOutput withRoll(double roll) {
        return withRoll((float) roll);
    }

    public ControlsOutput withThrottle(float throttle) {
        this.throttle = clamp(throttle);
        return this;
    }
    public ControlsOutput withThrottle(double throttle) {
        return withThrottle((float) throttle);
    }

    public ControlsOutput withJump(boolean jumpDepressed) {
        this.jumpDepressed = jumpDepressed;
        return this;
    }

    public ControlsOutput withBoost(boolean boostDepressed) {
        this.boostDepressed = boostDepressed;
        return this;
    }

    public ControlsOutput withHandbrake(boolean handbrakeDepressed) {
        this.handbrakeDepressed = handbrakeDepressed;
        return this;
    }

    public ControlsOutput withUseItem(boolean useItemDepressed) {
        this.useItemDepressed = useItemDepressed;
        return this;
    }

    public ControlsOutput withJump() {
        this.jumpDepressed = true;
        return this;
    }

    public ControlsOutput withBoost() {
        this.boostDepressed = true;
        return this;
    }

    public ControlsOutput withHandbrake() {
        this.handbrakeDepressed = true;
        return this;
    }

    public ControlsOutput withUseItem() {
        this.useItemDepressed = true;
        return this;
    }

    private float clamp(float value) {
        return Math.max(-1, Math.min(1, value));
    }

    @Override
    public float getSteer() {
        return steer;
    }

    @Override
    public float getThrottle() {
        return throttle;
    }

    @Override
    public float getPitch() {
        return pitch;
    }

    @Override
    public float getYaw() {
        return yaw;
    }

    @Override
    public float getRoll() {
        return roll;
    }

    @Override
    public boolean holdJump() {
        return jumpDepressed;
    }

    @Override
    public boolean holdBoost() {
        return boostDepressed;
    }

    @Override
    public boolean holdHandbrake() {
        return handbrakeDepressed;
    }

    @Override
    public boolean holdUseItem() {
        return useItemDepressed;
    }
}
