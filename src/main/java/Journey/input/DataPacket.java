package Journey.input;

import Journey.prediction.BallPredictionHelper;
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

    public final float t, dt;
    public final boolean isRoundActive, isKickoffPause, isKickoff, newBallTouch;
    private BallPredictionHelper.BPslice[] bPred;
    public BallPredictionHelper.BPslice[] bPred() {
        if(bPred == null)
            bPred = BallPredictionHelper.getPred();
        return bPred;
    }

    // Car position relative to the ball and ball's orientation (car orientation if ball not moving horizontally)
    public double cRelX, cRelY; // x>0 -> car to left of ball; y>0 -> car in front of ball

    // Abbreviations for commonly used data
    public final Vector3 cP, cV, cO, bP, bV;                    // Car position, velocity, orientation; Ball position, velocity
    public final Vector2 cPf, cVf, cOf, bPf, bVf;               // Flat versions (z=0)
    public final double cForwardVf, cVMag, cVfMag, bVMag, bVfMag; // Magnitudes

    public static final double relativeCoordsXRotator = Math.PI / 2.0;
    public DataPacket(GameTickPacket request, int playerIndex, DataPacket lastDP) {
        GameInfo gI = request.gameInfo();
        t = gI.secondsElapsed();
        dt = lastDP == null ? 0 : t - lastDP.t;
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

        Vector2 ballToCar = cPf.minus(bPf);
        if(bVf.isZero()) {
            cRelY = ballToCar.dotProduct(cOf);
            cRelX = ballToCar.dotProduct(cOf.rotateBy(relativeCoordsXRotator));
        } else {
            cRelY = ballToCar.dotProduct(bVf.normalized());
            cRelX = ballToCar.dotProduct(bVf.normalized().rotateBy(relativeCoordsXRotator));
        }

        isKickoff = Math.abs(bP.x) < 0.1 && Math.abs(bP.y) < 0.1 && Math.abs(bVMag) < 0.1; // Based exclusively on ball

        //Not first frame AND there has been a touch AND (there hadn't been a touch yet last frame OR this frame's latest touch is newer than last frame's)
        newBallTouch = lastDP != null && this.ball.latestTouch != null && (lastDP.ball.latestTouch == null || this.ball.latestTouch.gameSeconds > lastDP.ball.latestTouch.gameSeconds);
    }
}
