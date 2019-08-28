package Journey.prediction;

import rlbot.cppinterop.RLBotDll;
import rlbot.flat.BallPrediction;
import rlbot.flat.PredictionSlice;
import rlbot.render.Renderer;
import Journey.vector.Vector3;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

public class BallPredictionHelper {
    public static void drawTillMoment(float gameSeconds, Color color, Renderer renderer, int sliceResolution) {
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
    public static void drawTillMoment(float gameSeconds, Color color, Renderer renderer) {
        drawTillMoment(gameSeconds, color, renderer, 4);
    }
    public static void draw(Renderer renderer) {
        drawTillMoment(Float.MAX_VALUE, Color.cyan, renderer);
    }

    public static final class BPslice {
        Vector3 p;  // Position
        Vector3 v;  // Velocity
        float t;    // Game seconds
        public BPslice(PredictionSlice slice){
            this.p = new Vector3(slice.physics().location());
            this.v = new Vector3(slice.physics().velocity());
            this.t = slice.gameSeconds();
        }
    }
    public static BPslice[] getPred(int stepSize) {
        BallPrediction bP;
        try {
            bP = RLBotDll.getBallPrediction();
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
}
