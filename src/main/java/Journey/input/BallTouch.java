package Journey.input;

import Journey.util.Misc;
import rlbot.flat.Touch;
import Journey.vector.Vector3;

import java.util.List;

public class BallTouch {
    public final float gameSeconds; // The game time of the touch at 120hz. May be the time of the previous frame (if the touch occurred <(1/240)s after last frame?)
    public final Vector3 location;  // The absolute location of the touch (contact point on the surface of the ball)
    public final Vector3 ballPos;   // The position of the ball on contact
    public final int playerIndex;   // Index of the player that touched the ball
    public final String playerName; // Name of the player that touched the ball
    public final int team;          // Team of the player that touched the ball

    private BallTouch(final Touch touch) {
        this.gameSeconds = touch.gameSeconds();
        this.location = new Vector3(touch.location());
        this.ballPos = location.plus((new Vector3(touch.normal())).scaled(Misc.ballR));
        this.playerIndex = touch.playerIndex();
        this.playerName = touch.playerName();
        this.team = touch.team();
    }

    public static BallTouch getBallTouch(final Touch touch) {
        if(touch == null)
            return null;
        return new BallTouch(touch);
    }
}
