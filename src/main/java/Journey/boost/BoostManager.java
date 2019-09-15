package Journey.boost;

import Journey.input.DataPacket;
import Journey.vector.Vector2;
import rlbot.cppinterop.RLBotDll;
import rlbot.flat.BoostPadState;
import rlbot.flat.FieldInfo;
import rlbot.flat.GameTickPacket;
import Journey.vector.Vector3;

import java.io.IOException;
import java.util.ArrayList;

/**
 * Information about where boost pads are located on the field and what status they have.
 *
 * This class is here for your convenience, it is NOT part of the framework. You can change it as much
 * as you want, or delete it.
 */
public class BoostManager {

    private static final ArrayList<BoostPad> orderedBoosts = new ArrayList<>();
    private static final ArrayList<BoostPad> fullBoosts = new ArrayList<>();
    private static final ArrayList<BoostPad> smallBoosts = new ArrayList<>();

    public static ArrayList<BoostPad> getFullBoosts() {
        return fullBoosts;
    }

    public static ArrayList<BoostPad> getSmallBoosts() {
        return smallBoosts;
    }

    private static void loadFieldInfo(FieldInfo fieldInfo) {

        synchronized (orderedBoosts) {

            orderedBoosts.clear();
            fullBoosts.clear();
            smallBoosts.clear();

            for (int i = 0; i < fieldInfo.boostPadsLength(); i++) {
                rlbot.flat.BoostPad flatPad = fieldInfo.boostPads(i);
                BoostPad ourPad = new BoostPad(new Vector3(flatPad.location()), flatPad.isFullBoost());
                orderedBoosts.add(ourPad);
                if (ourPad.isFullBoost()) {
                    fullBoosts.add(ourPad);
                } else {
                    smallBoosts.add(ourPad);
                }
            }
        }
    }

    public static void loadGameTickPacket(GameTickPacket packet) {

        if (packet.boostPadStatesLength() > orderedBoosts.size()) {
            try {
                loadFieldInfo(RLBotDll.getFieldInfo());
            } catch (IOException e) {
                e.printStackTrace();
                return;
            }
        }

        for (int i = 0; i < packet.boostPadStatesLength(); i++) {
            BoostPadState boost = packet.boostPadStates(i);
            BoostPad existingPad = orderedBoosts.get(i); // existingPad is also referenced from the fullBoosts and smallBoosts lists
            existingPad.setActive(boost.isActive());
        }
    }

    public static final double boostPickupMaxEndV = -1;
    public static final double boostPickupMaxDist = 500;
    public static final double boostPickupMaxHeadingErr = 55;//45?
    public static BoostPad getTargetFullBoostPad(DataPacket dp) { // Null if shouldn't go for any full boosts
        BoostPad bestBP = null;
        //double bestTotalTripTime;
        for(int i = 0; i< BoostManager.getFullBoosts().size(); i++) {
            BoostPad currentBP = BoostManager.getFullBoosts().get(i);
            if(!currentBP.isActive())
                continue;
            //Vector2 bPloc = currentBP.getLocation().flatten();
            Vector2 bPTPL = currentBP.targetPickupLoc(dp.cPf);
            Vector2 carToBP = bPTPL.minus(dp.cPf);
            double carToBPMag = carToBP.magnitude();
            double carToBPHeadingErr = Math.abs(Math.toDegrees(carToBP.angleTo(dp.cOf)));
            if(carToBPMag < boostPickupMaxDist && carToBPHeadingErr < boostPickupMaxHeadingErr)
                return currentBP;
            //double travelTime = Misc.lineDriveDuration(dp.cVf.dotProduct(carToBP.normalized()), carToBPMag, dp.car.boost, boostPickupMaxEndV);
        }
        return bestBP;
    }
}
