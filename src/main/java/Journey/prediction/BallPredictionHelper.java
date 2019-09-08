package Journey.prediction;

import Journey.input.CarData;
import Journey.util.Misc;
import Journey.vector.Vector2;
import rlbot.cppinterop.RLBotDll;
import rlbot.flat.BallPrediction;
import rlbot.flat.PredictionSlice;
import rlbot.render.Renderer;
import Journey.vector.Vector3;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

public class BallPredictionHelper {
    public static void drawUntilMoment(Renderer renderer, float gameSeconds, Color color, int sliceResolution) {
        BPslice[] bP = getPred(sliceResolution);
        Vector3 previousLocation = null;
        for (int i = 0; i < bP.length; i++) {
            BPslice slice = bP[i];
            if (slice.t > gameSeconds) {
                break;
            }
            if (previousLocation != null) {
                renderer.drawLine3d(color, previousLocation, slice.p);
            }
            previousLocation = slice.p;
        }
    }
    public static void drawUntilMoment(Renderer renderer, float gameSeconds, Color color) {
        drawUntilMoment(renderer, gameSeconds, color, 4);
    }
    public static void draw(Renderer renderer) {
        drawUntilMoment(renderer, Float.MAX_VALUE, Color.cyan);
    }

    public static final class BPslice {
        public PredictionSlice slice;   // Framework PredictionSlice object
        public Boolean right;           // Car corner to hit with
        public Vector3 p;               // Position
        public Vector3 v;               // Velocity
        public float t;                 // Game seconds
        public BPslice(PredictionSlice slice) {
            if(slice != null) {
                this.slice = slice;
                this.p = new Vector3(slice.physics().location());
                this.v = new Vector3(slice.physics().velocity());
                this.t = slice.gameSeconds();
            }
        }
        public BPslice(PredictionSlice slice, Boolean right, Vector3 p, Vector3 v, float t){
            this(slice);
            this.right = right;
            this.p = p;
            this.v = v;
            this.t = t;
        }
        public BPslice withSide(boolean right) {
            return new BPslice(this.slice, right, this.p, this.v, this.t); // Automatically promotes to Boolean
        }
    }
    public static BPslice[] getPred(int stepSize) {
        BallPrediction bP;
        try {
            long t0 = System.nanoTime();
            bP = RLBotDll.getBallPrediction();
            long t1 = System.nanoTime();
            if(false && t1 - t0 > 250000)
                System.out.println("\n[BallPredictionHelper.getPred] RLBotDll.getBallPrediction() took " + ((t1 - t0) / 1000.0) + "us (>250)\n");
        } catch(rlbot.cppinterop.RLBotInterfaceException e) {
            System.out.println("[BallPredictionHelper.getPred] Couldn't get ball prediction!");
            e.printStackTrace();
            return new BPslice[0];
        }

        int len = bP.slicesLength();
        List<BPslice> outputAR = new ArrayList<>();
        stepSize = Math.max(1, stepSize);
        for(int i=0; i<len; i+=stepSize) {
            outputAR.add(new BPslice(bP.slices(i)));
        }
        return outputAR.toArray(new BPslice[0]);
    }
    public static BPslice[] getPred() {
        return getPred(1);
    }

    public static double posErr(BPslice[] bP, BPslice slice) {
        int i = 0;
        while(slice.t > bP[i].t + (Misc.frameT / 2.0)) {
            if(i + 1 >= bP.length) {
                System.out.println("[BallPredictionHelper.posErr] Exhausted bP (i=" + i + "); now: " + bP[0].t + ", slice.t: " + slice.t + ", bP[i].t: " + bP[i].t);
                break;
            }
            i++;
        }
        return bP[i].p.distance(slice.p);
    }
    public static double posErr(BPslice slice) {
        return posErr(getPred(), slice);
    }

    public static BPslice firstReachableSlice(Vector2 carPosition, Vector2 carVelocity, Vector2 carOrientationNose, double boost) { // position to ball surface
        BPslice[] bP = getPred();
        float now = bP[0].t;
        Vector2 leftCorner = carPosition.plus(Misc.carFrontCornerOffset2D(false).rotateBy(carOrientationNose)); // Rotate and add offset
        Vector2 rightCorner = carPosition.plus(Misc.carFrontCornerOffset2D(true).rotateBy(carOrientationNose)); // Rotate and add offset
        for (int i = 1; i < bP.length; i++) {
            if(bP[i].p.z > Misc.octaneFrontTopHeight + Misc.ballR) //TODO: Change eventually for dodges
                continue;

            Vector2 positionToBallL = bP[i].p.flatten().minus(leftCorner);
            Vector2 positionToBallR = bP[i].p.flatten().minus(rightCorner);
            double dL = positionToBallL.magnitude() - Misc.ballR;
            double dR = positionToBallR.magnitude() - Misc.ballR;

            //TODO: Change eventually for dodges
            double avgRequiredStraightLineSpd = Math.min(dL, dR) / (bP[i].t - now);
            if(avgRequiredStraightLineSpd > 2300)
                continue;

            double viL = carVelocity.dotProduct(positionToBallL.normalized());
            double viR = carVelocity.dotProduct(positionToBallR.normalized());
            //long t2 = System.nanoTime();
            double approachTL = Misc.lineDriveDuration(viL, dL, boost);
            double approachTR = Misc.lineDriveDuration(viR, dR, boost);
            //System.out.println("fRS t2 took " + ((System.nanoTime() - t2) / 1000000.0) + "ms");
            boolean right = approachTR < approachTL;
            if(now + (right ? approachTR : approachTL) < bP[i].t) {
                //System.out.println(i + ": d=" + d + ", vi=" + vi + ", t=" + approachT);
                //System.out.println("fRS to " + i + " took " + ((System.nanoTime() - t0) / 1000000.0) + "ms");
                return bP[i].withSide(right);
            }
        }
        //System.out.println("fRS reached end (failure), took " + ((System.nanoTime() - t0) / 1000000.0) + "ms");
        return null;
    }
    public static BPslice firstReachableSlice(CarData car) {
        return firstReachableSlice(car.position.flatten(), car.velocity.flatten(), car.orientation.noseVector.flatten(), car.boost);
    }
}
