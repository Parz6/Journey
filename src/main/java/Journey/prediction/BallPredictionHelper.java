package Journey.prediction;

import rlbot.flat.BallPrediction;
import rlbot.flat.PredictionSlice;
import rlbot.render.Renderer;
import Journey.vector.Vector3;

import java.awt.Color;

public class BallPredictionHelper {
    public static void drawTillMoment(BallPrediction bP, float gameSeconds, Color color, Renderer renderer, int sliceResolution) {
        Vector3 previousLocation = null;
        int bPlen = bP.slicesLength();
        for (int i = 0; i < bPlen; i += sliceResolution) {
            PredictionSlice slice = bP.slices(i);
            if (slice.gameSeconds() > gameSeconds) {
                break;
            }
            Vector3 location = new Vector3(slice.physics().location());
            if (previousLocation != null) {
                renderer.drawLine3d(color, previousLocation, location);
            }
            previousLocation = location;
        }
    }
    public static void drawTillMoment(BallPrediction bP, float gameSeconds, Color color, Renderer renderer) {
        drawTillMoment(bP, gameSeconds, color, renderer, 4);
    }
    public static void draw(BallPrediction bP, Renderer renderer) {
        drawTillMoment(bP, Float.MAX_VALUE, Color.cyan, renderer);
    }
}
