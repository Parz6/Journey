package Journey;

import Journey.input.DataPacket;
import Journey.output.ControlsOutput;
import Journey.prediction.BallPredictionHelper;
import Journey.prediction.BallPredictionHelper.BPslice;
import Journey.util.Misc;
import Journey.vector.Vector2;
import Journey.vector.Vector3;
import rlbot.cppinterop.RLBotDll;
import rlbot.gamestate.*;

import static Journey.util.Misc.clamp;

public class Testing {
    private Journey bot;

    public Testing(Journey bot) {
        this.bot = bot;
    }

    public static void scenario1(int carI) { // Carry with turn
        float cX = -2300, cY = -4000;
        float cRelY = -30;
        float carHeading = (float) Math.toRadians(0);
        float carSpd = 400;
        float cVX = carSpd * (float) Math.cos(carHeading), cVY = carSpd * (float) Math.sin(carHeading);

        float bX = cX + (cRelY * (float) Math.cos(carHeading));
        float bY = cY + (cRelY * (float) Math.sin(carHeading));
        float ballZ = (float) (300 + Misc.ballR + Misc.octaneFrontTopHeight + 5);
        float ballDirection = (float) Math.toRadians(0);
        float ballSpd = 100 + carSpd;
        float bVX = ballSpd * (float) Math.cos(ballDirection), bVY = ballSpd * (float) Math.sin(ballDirection);
        GameState gameState = new GameState()
                .withCarState(carI, new CarState() //Bot
                        .withBoostAmount(100F)
                        .withPhysics(new PhysicsState()
                                .withLocation(new DesiredVector3(cX, cY, (float) (Misc.octaneJointHeight + 5)))
                                .withVelocity(new DesiredVector3(cVX, cVY, 0F))
                                .withRotation(new DesiredRotation(0F, carHeading, 0F))))
                .withBallState(new BallState()
                        .withPhysics(new PhysicsState()
                                .withLocation(new DesiredVector3(bX, bY, ballZ))
                                .withVelocity(new DesiredVector3(bVX, bVY, 0F))
                                .withRotation(new DesiredRotation(0F, 0F, 0F))));

        RLBotDll.setGameState(gameState.buildPacket());
    }

    public static void scenario2(int carI) { // Testing cPFRS_getVelocitySLRotator()
        float cX = 3500, cY = -500;
        float carHeading = (float) Math.toRadians(-135);
        float carSpd = 400;
        float cVX = carSpd * (float) Math.cos(carHeading), cVY = carSpd * (float) Math.sin(carHeading);

        float bX = 3500, bY = -3000;
        float ballZ = (float) Misc.ballR + 1;
        float ballDirection = (float) Math.toRadians(150);
        float ballSpd = 1500;//1500
        float bVX = ballSpd * (float) Math.cos(ballDirection), bVY = ballSpd * (float) Math.sin(ballDirection);
        GameState gameState = new GameState()
                .withCarState(carI, new CarState() //Bot
                        .withBoostAmount(100F)
                        .withPhysics(new PhysicsState()
                                .withLocation(new DesiredVector3(cX, cY, (float) (Misc.octaneJointHeight + 5)))
                                .withVelocity(new DesiredVector3(cVX, cVY, 0F))
                                .withRotation(new DesiredRotation(0F, carHeading, 0F))))
                .withBallState(new BallState()
                        .withPhysics(new PhysicsState()
                                .withLocation(new DesiredVector3(bX, bY, ballZ))
                                .withVelocity(new DesiredVector3(bVX, bVY, 0F))
                                .withRotation(new DesiredRotation(0F, 0F, 0F))));

        RLBotDll.setGameState(gameState.buildPacket());
    }

    public static BPslice nextPostBounceSlice(DataPacket dp, double startT, double vZMin) {
        BPslice[] bP = dp.bPred();
        BPslice prevSlice = bP[0];
        for(int i=1; i<bP.length; i++) {
            if(bP[i].t < startT)
                continue;
            if(bP[i].v.z > vZMin && prevSlice.v.z < vZMin) {
                //System.out.println("[Testing.nextPostBounceSlice] to " + i + " took " + ((System.nanoTime() - t0) / 1000000.0) + "ms");
                return bP[i];
            }
            prevSlice = bP[i];
        }
        //System.out.println("[Testing.nextPostBounceSlice] failed, took " + ((System.nanoTime() - t0) / 1000000.0) + "ms");
        return null; // None found
    }
    public static BPslice nextPostBounceSliceAfter(DataPacket dp, double startT) {
        return nextPostBounceSlice(dp, startT, 40);
    }
    public static BPslice nextDribblablePostBounceSliceAfter(DataPacket dp, double startT) {
        return nextPostBounceSlice(dp, startT, 100);
    }
    public static BPslice nextPostBounceSlice(DataPacket dp) {
        return nextPostBounceSliceAfter(dp, -1);
    }
    public static BPslice nextDribblablePostBounceSlice(DataPacket dp) {
        return nextPostBounceSliceAfter(dp, -1);
    }

    public void debugNBT(DataPacket dp, DataPacket lastDP) {
        if(false && dp.newBallTouch) {
            Vector3 lTL = dp.ball.latestTouch.location;
            float lTT = dp.ball.latestTouch.gameSeconds;
            System.out.println("                                                                 Last frame, \t\t\tCurrent frame");
            System.out.println("Touch.location() to frame ball positions:                        " + lTL.distance(lastDP.bP) + ", \t\t" + lTL.distance(dp.bP));
            System.out.println("Extrapolated latest touch ball position to frame ball positions: " + dp.ball.latestTouch.ballPos.distance(lastDP.bP) + ", \t" + dp.ball.latestTouch.ballPos.distance(dp.bP));
            System.out.println("Touch time to frame time diffs:                                  " + ((lTT - lastDP.t) / 1000.0) + ", \t\t" + ((dp.t - lTT) / 1000.0));

            double car2ball = dp.cPf.distance(dp.bPf);
            double car2ballLast = lastDP.cPf.distance(lastDP.bPf);
            System.out.println("\n" + car2ball + ", " + car2ballLast + "\n\n");
        }
    }

    BPslice dest;
    Vector2 carDest;
    private boolean getDest(DataPacket dp) {
        BPslice fRS = dp.car.fRS(dp);
        if(fRS == null)
            return false;
        BPslice nDPBSA = Testing.nextDribblablePostBounceSliceAfter(dp, fRS.t);
        if(nDPBSA == null)
            return false;
        //boolean rightCorner = dp.cOf.angleTo(nDPBSA.p.minus()) < 0;
        //System.out.println("fRS right? " + fRS.right);

        dest = nDPBSA;
        carDest = Misc.carDestForFrontCornerContact(nDPBSA.p.flatten(), Misc.carFrontCorner(dp.car, fRS.right), fRS.right);
        return true;
    }
    public ControlsOutput hitBounce(DataPacket dp) {
        boolean recalcDest;

        if(dest == null ) {
            System.out.println("[Testing.hitBounce] ##### dest null #####");
            recalcDest = true;
        } else if(dp.newBallTouch) {
            System.out.println("[Testing.hitBounce] ##### newBallTouch #####");
            recalcDest = true;
        } else if(BallPredictionHelper.posErr(dp, dest) > 10) {
            double slicePosErr = BallPredictionHelper.posErr(dp, dest);
            System.out.println("\n[Testing.hitBounce] ##### dest inaccurate, bP changed? slicePosErr: " + slicePosErr + " #####\n");
            recalcDest = true;
        } else {
            Vector2 carToDest = carDest.minus(dp.cPf);
            double headingErrToDest = Math.abs(Math.toDegrees(dp.cOf.angleTo(carToDest)));

            double slicePosErr = BallPredictionHelper.posErr(dp, dest);
            System.out.println("[Testing.hitBounce] slicePosErr: " + slicePosErr + ", headingErrToDest: " + headingErrToDest);

            recalcDest = headingErrToDest > 15 + clamp(300 - carToDest.magnitude(), 0, 90);
            if(recalcDest)
                System.out.println("[Testing.hitBounce] recalcDest true, headingErrToDest too high");
        }
        if(recalcDest && !getDest(dp)) { // Need recalc but cannot
            System.out.println("[Testing.hitBounce] Cannot recalc!");
            return new ControlsOutput();
        } else {
            if(dest == null) {
                System.out.println("[Testing.hitBounce] Dest unexpectedly null!");
                return new ControlsOutput();
            }
            return Misc.spinLineDrive(dp, carDest, dest.t - dp.t, -1); //TODO: Change maxEndV
        }
    }
}
