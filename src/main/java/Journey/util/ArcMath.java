package Journey.util;

import Journey.vector.Vector2;

public class ArcMath {
    private ArcMath() {} // Disable instantiation

    // * 2.0 for heading change (radians of circle, out of 2pi)
    // Acute positive angle (radians) from chord (start to end) to tangent of start point for chord length x across circle of radius r
    public static double acuteHeadingOffsetForArcTurn(Vector2 start, Vector2 end, double r) { // Acute heading offset from start -> end aka end.minus(start)
        double x = start.distance(end);
        r = Math.max(x / 2.0, r); // Forces x: [0, 2r]; r < x/2 is an impossible situation (chord longer than diameter)
        return Math.asin(x / (2.0 * r)); // arcsin(x/2r): [0, pi/2] for x: [0, 2r]
    }
    // It doesn't matter what the start orientation is; there's only one radius given start and end positions and ending orientation
    public static double radiusForArcTurn(Vector2 startPf, Vector2 endPf, Vector2 endOf) {
        Vector2 displacement = endPf.minus(startPf); //Displacement between start and end points
        double distance = displacement.magnitude(); //Distance d between start and end points
        double theta1 = displacement.angleTo(endOf); //Shot angle
        //double theta = 2.0 * theta1; //Arc angle
        //while (theta < 0)
        //    theta += Math.PI * 2.0;
        double radius = distance / (2.0 * Math.sin(theta1)); //d / (2.0 * Math.sin(theta))
        return Math.abs(radius);
    }
    public static double arcTurnDuration(double radius, double thetaRad, double v) {
        return thetaRad * radius / v; // ((thetaRad / 2*pi) * (2*pi*r)) / v
    }
    public static double arcTurnDuration(double radius, double thetaRad) { // Assumes max possible speed for radius
        return arcTurnDuration(radius, thetaRad, Misc.speedForTurnRadius(radius));
    }
    public static double avgSpeedForArcTurnDRT(double distance, double radius, double timeAllotted) {
        double arcLen = 2.0 * radius * Math.asin(distance / (2.0 * radius)); // distance is chord length
        return arcLen / timeAllotted;
    }
    public static double minRadiusForArcTurn(Vector2 startPf, Vector2 endPf, double maxHeadingChange) { // Minor arcs only (<pi); returns -1 for straight line (r=infinity)
        if(maxHeadingChange == 0)
            return -1;
        maxHeadingChange = Math.abs(maxHeadingChange);
        Vector2 startToEnd = endPf.minus(startPf);
        if(maxHeadingChange >= Math.PI)
            return startToEnd.magnitude();
        Vector2 endOfLimit = startToEnd.rotateBy(maxHeadingChange / 2.0);
        return Math.max(startToEnd.magnitude() / 2.0, radiusForArcTurn(startPf, endPf, endOfLimit));
    }

    // Returns: {arcEnd to locationArcEndFaces, arc % of circle as an angle} OR null if impossible (radius too big)
    public static Vector2[] vectorToArcEnd(Vector2 arcStart, Vector2 locationArcStartFaces, Vector2 locationArcEndFaces, double arcRadius) { //Vector from arc end to locationArcEndFaces
        Vector2 straightawayToCar = locationArcEndFaces.minus(arcStart); //Originally carLocRel
        Vector2 strikeToStraightaway = arcStart.minus(locationArcStartFaces);
        double rotatorRad = strikeToStraightaway.angleTo(straightawayToCar);
        Vector2 carLocRelRotated = (new Vector2(0, straightawayToCar.magnitude())).rotateBy(Math.abs(rotatorRad));
        boolean arcToRight = rotatorRad < 0;
        Vector2 finalCarLoc = carLocRelRotated.plus(new Vector2(arcRadius, 0));
        Vector2[] absoluteResultArr = absoluteVectorToArcEnd(finalCarLoc, arcRadius);
        if (absoluteResultArr == null) //Impossible with given turn radius
            return null;
        Vector2[] absoluteResult = absoluteResultArr;
        if (arcToRight) {
            absoluteResult[0] = new Vector2(-absoluteResult[0].x, absoluteResult[0].y); //Un-mirror over y
            absoluteResult[1] = (new Vector2(absoluteResult[1].x, -absoluteResult[1].y)).normalized(); //TODO: Is this right? Should it not be done at all?
        }
        Vector2[] result = new Vector2[2];
        result[0] = absoluteResult[0].rotateBy(strikeToStraightaway.angle() - (Math.PI / 2.0)); //Result (rotated back)
        result[1] = absoluteResultArr[1]; //Theta (of arc) as vector
        return result;
    }
    // Assuming arc of radius starts at (radius, 0) facing 90deg (positive y) and ends pointing towards target; Null if impossible (radius too big)
    private static Vector2[] absoluteVectorToArcEnd(Vector2 target, double radius) { // Target to arc end
        double x = target.x;
        double y = target.y;
        double radical = Math.sqrt(-Math.pow(radius, 2) + Math.pow(x, 2) + Math.pow(y, 2));
        double thetaNeg = 2.0 * Math.atan((y - radical) / (x + radius)); //+radical instead for clockwise (wrong way)
        //double slope = -1.0 / Math.tan(thetaNeg);
        if (Double.isNaN(thetaNeg)) //Impossible with given turn radius
            return null;
        Vector2 thetaLocVec = new Vector2(radius * Math.cos(thetaNeg), radius * Math.sin(thetaNeg));
        Vector2[] result = new Vector2[2];
        result[0] = thetaLocVec.minus(target); //Result
        result[1] = new Vector2(Math.cos(thetaNeg), Math.sin(thetaNeg)); //Theta as vector
        return result;
    }
}
