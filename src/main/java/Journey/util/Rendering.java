package Journey.util;

import Journey.vector.Vector2;
import Journey.vector.Vector3;
import rlbot.render.Renderer;

import java.awt.Color;

public class Rendering {
    public static void renderMinor2dArc(Renderer renderer, Vector2 startPf, Vector2 endPf, Vector2 endOf, double drawZ, Color color, boolean drawGuides) {
        endOf = endOf.normalized();
        final double lineLength = 50;
        double radius = Misc.radiusForTurn(startPf, endPf, endOf);
        boolean clockwise = endOf.angleTo(endPf.minus(startPf)) < 0; // From end to start, top-down
        Vector2 coc = endPf.plus(endOf.scaledToMagnitude(radius).rotateBy((clockwise ? 1.0 : -1.0) * (Math.PI / 2.0))); // Center of circle

        if(drawGuides) {
            renderer.drawCenteredRectangle3d(Color.red, coc.inflate(drawZ), 20, 20, true);
            renderer.drawCenteredRectangle3d(Color.red, startPf.inflate(drawZ), 20, 20, true);
            renderer.drawCenteredRectangle3d(Color.red, endPf.inflate(drawZ), 20, 20, true);
            renderer.drawLine3d(Color.blue, coc.inflate(drawZ), startPf.inflate(drawZ));
            renderer.drawLine3d(Color.blue, coc.inflate(drawZ), endPf.inflate(drawZ));
        }

        // Generate points, from end to start, to draw straight lines between
        Vector2 lastPoint = endPf;
        double radiansPerLineSegment = (clockwise ? -1.0 : 1.0) * (lineLength / radius); // 2 pi * (lineLength / (2 pi radius))
        double startAngle = endPf.minus(coc).angle(); // Radians
        //double angleRange = endPf.minus(coc).angleTo(startPf.minus(coc)); // Radians, only works up to +-PI
        double angleRange = 2.0 * endPf.minus(startPf).angleTo(endOf); // Radians
        double drawnAngle = 0; //Radians total
        while(Math.abs(drawnAngle) < Math.PI * 2.0) {
            drawnAngle += radiansPerLineSegment;
            if (Math.abs(drawnAngle) > Math.abs(angleRange)) {
                renderer.drawLine3d(color, lastPoint.inflate(drawZ), startPf.inflate(drawZ));
                break;
            }
            double actualDrawAngle = startAngle + drawnAngle;
            Vector2 relativePoint = (new Vector2(Math.cos(actualDrawAngle), Math.sin(actualDrawAngle))).scaledToMagnitude(radius); // Relative to coc
            Vector2 drawPoint = coc.plus(relativePoint);
            if(drawPoint.magnitude() < 7000)
                renderer.drawLine3d(color, lastPoint.inflate(drawZ), drawPoint.inflate(drawZ));
            lastPoint = drawPoint;
        }
    }
    public static void renderMinor2dArc(Renderer renderer, Vector2 startPf, Vector2 endPf, Vector2 endOf, double drawZ, Color color) {
        renderMinor2dArc(renderer, startPf, endPf, endOf, drawZ, color);
    }

    private static final double defaultImpactRadius = 50.0;
    public static void drawImpact(Renderer renderer, Vector3 loc, double radius, Color color) {
        Vector3[] outerPos = new Vector3[6];
        outerPos[0] = new Vector3(1, 0, 0);
        outerPos[1] = new Vector3(0, 1, 0);
        outerPos[2] = new Vector3(0, 0, 1);
        outerPos[3] = new Vector3(1, 1, 1);
        outerPos[4] = new Vector3(1, -1, 1);
        outerPos[5] = new Vector3(-1, -1, 1);
        outerPos[5] = new Vector3(-1, 1, 1);
        for(int i=0; i<outerPos.length; i++) {
            if(i < 3 || true)
                outerPos[i] = outerPos[i].scaledToMagnitude(radius);
            else // Make diagonals smaller
                outerPos[i] = outerPos[i].scaledToMagnitude(radius / 2.0);
            renderer.drawLine3d(color, loc.minus(outerPos[i]), loc.plus(outerPos[i]));
        }
    }
    public static void drawImpact(Renderer renderer, Vector3 loc, Color color) {
        drawImpact(renderer, loc, defaultImpactRadius, color);
    }
}
