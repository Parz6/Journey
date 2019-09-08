package Journey.input;


import rlbot.flat.BallInfo;
import Journey.vector.Vector3;

public class BallData {
    public final Vector3 position;
    public final Vector3 velocity;
    public final Vector3 spin;
    public final BallTouch latestTouch; // null if framework ball touch is null

    public BallData(final BallInfo ball, final BallTouch latestTouch) {
        this.position = new Vector3(ball.physics().location());
        this.velocity = new Vector3(ball.physics().velocity());
        this.spin = new Vector3(ball.physics().angularVelocity());
        this.latestTouch = latestTouch;
    }
}
