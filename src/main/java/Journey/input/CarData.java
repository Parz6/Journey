package Journey.input;

import Journey.prediction.BallPredictionHelper;
import Journey.vector.Vector3;

public class CarData {
    /** The location of the car on the field. (0, 0, 0) is center field. */
    public final Vector3 position;

    /** The velocity of the car. */
    public final Vector3 velocity;

    /** The orientation of the car */
    public final CarOrientation orientation;

    /** Boost ranges from 0 to 100 */
    public final double boost;

    /** True if the car is driving on the ground, the wall, etc. In other words, true if you can steer. */
    public final boolean hasWheelContact;

    /**
     * True if the car is showing the supersonic and can demolish enemies on contact.
     * This is a close approximation for whether the car is at max speed.
     */
    public final boolean isSupersonic;
    public final boolean isDemolished;
    public final boolean jumped;
    public final boolean doubleJumped;
    public final boolean isBot;
    public final int index;
    public final String name;

    /**
     * 0 for blue team, 1 for orange team.
     */
    public final int team;

    private BallPredictionHelper.BPslice fRS;
    public BallPredictionHelper.BPslice fRS(DataPacket dp) {
        if(fRS == null) {
            fRS = BallPredictionHelper.firstReachableSlice(dp, this);
        }
        return fRS;
    }

    public CarData(rlbot.flat.PlayerInfo playerInfo, int index) {
        this.position = new Vector3(playerInfo.physics().location());
        this.velocity = new Vector3(playerInfo.physics().velocity());
        this.orientation = CarOrientation.fromFlatbuffer(playerInfo);
        this.boost = playerInfo.boost();
        this.isSupersonic = playerInfo.isSupersonic();
        this.isDemolished = playerInfo.isDemolished();
        this.jumped = playerInfo.jumped();
        this.doubleJumped = playerInfo.doubleJumped();
        this.isBot = playerInfo.isBot();
        this.index = index;
        this.name = playerInfo.name();
        this.team = playerInfo.team();
        this.hasWheelContact = playerInfo.hasWheelContact();
    }
}
