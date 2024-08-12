package org.rustlib.rustboard;

import static org.rustlib.config.Loader.defaultStorageDirectory;

import com.qualcomm.robotcore.util.RobotLog;

import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;
import org.rustlib.config.Loader;
import org.rustlib.core.RobotControllerActivity;

import java.io.File;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.UnknownHostException;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Timer;
import java.util.TimerTask;

import javax.json.Json;
import javax.json.JsonArray;
import javax.json.JsonArrayBuilder;
import javax.json.JsonObject;
import javax.json.JsonObjectBuilder;
import javax.json.JsonValue;

public class RustboardServer extends WebSocketServer {
    public static final int port = 5801;
    static final File rustboardStorageDir = new File(defaultStorageDirectory, "Rustboard");
    private static final File rustboardMetadataFile = new File(rustboardStorageDir, "rustboard_metadata.json");
    private static final File storedRustboardDir = new File(rustboardStorageDir, "rustboards");
    private static RustboardServer instance = null;
    private static boolean debugMode = false;
    private Rustboard activeRustboard;
    private final JsonObject rustboardMetaData;
    private final HashSet<String> storedRustboardIds = new HashSet<>();
    private final HashMap<String, Rustboard> loadedRustboards = new HashMap<>();
    private final Timer saveScheduler = new Timer();
    private static final int rustboardAutoSavePeriod = 30000;
    private static final int pingClientPeriod = 8000;
    private static final ClientUpdater clientUpdater = new ClientUpdater();

    ClientUpdater getClientUpdater() {
        return clientUpdater;
    }

    private void autoSave() {
        if (!RobotControllerActivity.opModeRunning()) {
            saveLayouts();
            saveScheduler.schedule(new TimerTask() {
                @Override
                public void run() {
                    autoSave();
                }
            }, rustboardAutoSavePeriod);
        }
    }

    public Rustboard getActiveRustboard() {
        if (activeRustboard == null) {
            activeRustboard = Rustboard.emptyRustboard();
        }
        return activeRustboard;
    }

    private RustboardServer(int port) throws UnknownHostException {
        super(new InetSocketAddress(port));
        JsonObject metaData;
        setReuseAddr(true);
        RobotLog.v("Rustboard server initialized");
        if (!rustboardStorageDir.exists()) {
            if (!rustboardStorageDir.mkdir())
                throw new RuntimeException("Failed to generate storage folders needed for the Rustboard server");
        }
        rustboardMetaData = Loader.safeLoadJsonObject(rustboardMetadataFile);
        if (rustboardMetaData.containsKey("rustboards")) {
            JsonArray dataArray = rustboardMetaData.getJsonArray("rustboards");
            for (JsonValue value : dataArray) {
                JsonObject data = (JsonObject) value;
                String uuid = data.getString("uuid");
                storedRustboardIds.add(uuid);
                if (data.getBoolean("active")) {
                    try {
                        activeRustboard = Rustboard.load(uuid);
                    } catch (Rustboard.NoSuchRustboardException e) {
                        e.printStackTrace();
                    }
                }
            }
        }
        RobotControllerActivity.onOpModeStop(this::autoSave);
        new Timer().scheduleAtFixedRate(new ClientUpdater(), 0, 50);
    }

    static void messageActiveRustboard(JsonObject json) {
        getInstance().activeRustboard.getConnection().send(json.toString());
    }

    public static void enableDebugMode() { // TODO: make a good way to enable and disable debug mode
        debugMode = false;
    }

    public static void disableDebugMode() {
        debugMode = true;
    }

    public static boolean inDebugMode() {
        return debugMode;
    }

    public static void log(Object value) { // TODO: create simpler logger

    }

    private static void clearStorage() throws IOException {
        File[] files = storedRustboardDir.listFiles();
        if (files != null) {
            for (File file : files) {
                if (!file.delete())
                    throw new IOException(String.format("Could not delete \"%s\"", file.getAbsolutePath()));
            }
        }
    }

    public static RustboardServer getInstance() {
        if (instance == null) {
            try {
                instance = new RustboardServer(port);
                RobotLog.v("dashboard server started");
            } catch (UnknownHostException e) {
                e.printStackTrace();
            }
        }
        return instance;
    }

    @Override
    public void start() {
        if (!debugMode) {
            try {
                super.start();
            } catch (IllegalStateException e) {
                log(e);
            }
        }
    }

    @Override
    public void stop() {
        saveLayouts();
        try {
            super.stop();
        } catch (IOException | InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void onStart() {
        setConnectionLostTimeout(3);
    }

    public static boolean isActiveRustboard() {
        return getInstance().activeRustboard != null && getInstance().activeRustboard.isConnected();
    }

    @Override
    public void onOpen(WebSocket conn, ClientHandshake handshake) {
        new Timer().scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                broadcast("{\"action\": \"ping\"}");
            }

        }, pingClientPeriod, pingClientPeriod);
    }

    @Override
    public void onClose(WebSocket conn, int code, String reason, boolean remote) {
        if (activeRustboard.getConnection() == conn) {
            activeRustboard.onDisconnect();
        }
        for (Rustboard rustboard : loadedRustboards.values()) {
            if (rustboard.isConnected()) {
                activeRustboard = rustboard;
                break;
            }
        }
        log("client " + conn.getRemoteSocketAddress().toString() + " disconnected from the robot.");
    }

    @Override
    @SuppressWarnings("ConstantConditions")
    public void onMessage(WebSocket conn, String message) {
        JsonObject messageJson = Loader.readJsonString(message);
        switch (messageJson.getString("action")) {
            case "client_details":
                Time.calibrateUTCTime(messageJson.getJsonNumber("utc_time").longValue());
                String uuid = messageJson.getString("uuid");
                JsonObject rustboardJson = messageJson.getJsonObject("rustboard");
                JsonArray clientNodes = messageJson.getJsonArray("nodes");
                Rustboard rustboard;
                if (loadedRustboards.containsKey(uuid)) {
                    rustboard = loadedRustboards.get(uuid).mergeWithClientRustboard(clientNodes);
                } else if (storedRustboardIds.contains(uuid)) {
                    try {
                        rustboard = Rustboard.load(uuid).mergeWithClientRustboard(clientNodes);
                    } catch (Rustboard.NoSuchRustboardException e) {
                        rustboard = new Rustboard(uuid, rustboardJson);
                    }
                } else {
                    rustboard = new Rustboard(uuid, rustboardJson);
                }
                if (isActiveRustboard()) {
                    rustboard.notifyClient("Rustboard queued because another Rustboard is connected", NoticeType.NEUTRAL, 5000);
                } else {
                    activeRustboard = rustboard;
                    JsonObjectBuilder builder = Json.createObjectBuilder();
                    builder.add("action", "set_active");
                    rustboard.getConnection().send(builder.build().toString());
                }
                rustboard.setConnection(conn);
                loadedRustboards.put(uuid, rustboard);

                break;
            case "save_path":
                Loader.safeWriteJson(new File(defaultStorageDirectory, messageJson.getString("node_id").replace(" ", "_") + ".json"), messageJson.getJsonObject("path"));
                break;
            case "save_value":
                Loader.safeWriteString(new File(defaultStorageDirectory, messageJson.getString("node_id").replace(" ", "_") + ".txt"), messageJson.getString("value"));
                break;
            case "exception":
                throw new RustboardException(messageJson.getString("exception_message"));
            default:
                if (activeRustboard != null) {
                    activeRustboard.onMessage(messageJson);
                }
        }
    }

    public void saveLayouts() {
        JsonObjectBuilder metadataBuilder = Json.createObjectBuilder(rustboardMetaData);
        JsonArrayBuilder rustboardArrayBuilder = Json.createArrayBuilder(rustboardMetaData.getJsonArray("rustboards"));
        metadataBuilder.add("rustboards", rustboardArrayBuilder);
        loadedRustboards.forEach((uuid, rustboard) -> {
                    JsonObjectBuilder rustboardDescriptor = Json.createObjectBuilder();
                    rustboardDescriptor.add("uuid", uuid);
                    rustboardDescriptor.add("active", rustboard == activeRustboard);
                    rustboardArrayBuilder.add(rustboardDescriptor);
                    rustboard.save(new File(rustboardStorageDir, uuid + ".json"));
                }
        );
    }

    @Override
    public void onError(WebSocket conn, Exception e) {
        log(e);
    }

    public static class RustboardException extends RuntimeException {
        public RustboardException(String message) {
            super(message);
        }
    }
}