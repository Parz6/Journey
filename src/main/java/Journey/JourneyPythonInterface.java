package Journey;

import rlbot.Bot;
import rlbot.manager.BotManager;
import rlbot.pyinterop.SocketServer;

public class JourneyPythonInterface extends SocketServer {

    public JourneyPythonInterface(int port, BotManager botManager) {
        super(port, botManager);
    }

    protected Bot initBot(int index, String botType, int team) {
        return new Journey(index);
    }
}
