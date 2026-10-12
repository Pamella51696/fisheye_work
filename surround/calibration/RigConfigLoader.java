package surround.calibration;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Map;

public final class RigConfigLoader {

    private RigConfigLoader() {
    }

    public static RigConfig load(Path path) throws IOException {
        if (!Files.isRegularFile(path)) {
            throw new IOException("rig config not found: " + path);
        }
        String json = Files.readString(path);
        Object root = JsonUtil.parse(json);
        return RigConfig.fromJsonMap(JsonUtil.asObject(root));
    }
}
