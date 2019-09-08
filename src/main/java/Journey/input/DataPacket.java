package Journey.input;

import Journey.vector.Vector2;
import Journey.vector.Vector3;
import rlbot.flat.GameInfo;
import rlbot.flat.GameTickPacket;

import java.util.ArrayList;
import java.util.List;

public class DataPacket {
    // Data
    public final CarData car;
    public final List<CarData> allCars;
    public final BallData ball;

    public final int team;
    public final int playerIndex;

    public final float t;
    public final boolean isRoundActive, isKickoffPause, isKickoff, newBallTouch;

    // Abbreviations for commonly used data
    public final Vector3 cP, cV, cO, bP, bV;                    // Car position, velocity, orientation; Ball position, velocity
    public final Vector2 cPf, cVf, cOf, bPf, bVf;               // Flat versions (z=0)
    public final double cForwardVf, cVMag, cVfMag, bVMag, bVfMag; // Magnitudes

    public DataPacket(GameTickPacket request, int playerIndex, DataPacket lastDP) {
        GameInfo gI = request.gameInfo();
        t = gI.secondsElapsed();
        isRoundActive = gI.isRoundActive();
        isKickoffPause = gI.isKickoffPause();

        allCars = new ArrayList<>();
        for (int i = 0; i < request.playersLength(); i++) {
            allCars.add(new CarData(request.players(i), i));
        }

        this.playerIndex = playerIndex;
        this.car = allCars.get(playerIndex);
        this.team = this.car.team;
        this.ball = new BallData(request.ball(), BallTouch.getBallTouch(request.ball().latestTouch()));

        cP = car.position;                  cPf = cP.flatten();
        cV = car.velocity;                  cVf = cV.flatten();
        cO = car.orientation.noseVector;    cOf = cO.flatten().normalized();
        bP = ball.position;                 bPf = bP.flatten();
        bV = ball.velocity;                 bVf = bV.flatten();
        cVMag = cV.magnitude();             cVfMag = cVf.magnitude();
        bVMag = bV.magnitude();             bVfMag = bVf.magnitude();
        cForwardVf = cVf.dotProduct(cOf);

        isKickoff = Math.abs(bP.x) < 0.1 && Math.abs(bP.y) < 0.1 && Math.abs(bVMag) < 0.1; // Based exclusively on ball

        //Not first frame AND there has been a touch AND (there hadn't been a touch yet last frame OR this frame's latest touch is newer than last frame's)
        newBallTouch = lastDP != null && this.ball.latestTouch != null && (lastDP.ball.latestTouch == null || this.ball.latestTouch.gameSeconds > lastDP.ball.latestTouch.gameSeconds);
    }
}
