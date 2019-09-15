package Journey.boost;


import Journey.vector.Vector2;
import Journey.vector.Vector3;

/**
 * Representation of one of the boost pads on the field.
 *
 * This class is here for your convenience, it is NOT part of the framework. You can change it as much
 * as you want, or delete it.
 */
public class BoostPad {

    private final Vector3 location;
    private final boolean isFullBoost;
    private boolean isActive;
    public static double pickupRadius = 208; //From boost pad center to car joint, assume pad as spawned before arrival. hitbox is cylindrical.

    public BoostPad(Vector3 location, boolean isFullBoost) {
        this.location = location;
        this.isFullBoost = isFullBoost;
    }

    public void setActive(boolean active) {
        isActive = active;
    }

    public Vector3 getLocation() {
        return location;
    }

    public boolean isFullBoost() {
        return isFullBoost;
    }

    public boolean isActive() {
        return isActive;
    }

    public static final double defaultPickupThreshold = 30;
    public Vector2 targetPickupLoc(Vector2 sourcePos, double pickupThreshold) {
        Vector2 padLoc = getLocation().flatten();
        return padLoc.plus(sourcePos.minus(padLoc).scaledToMagnitude(pickupRadius - pickupThreshold)) ;
    }
    public Vector2 targetPickupLoc(Vector2 sourcePos) {
        return targetPickupLoc(sourcePos, defaultPickupThreshold);
    }
}
