package frc.robot.util;

import com.fasterxml.jackson.databind.*;
import com.fasterxml.jackson.databind.node.*;
import java.nio.file.*;
import java.util.stream.*;

public class MirrorPaths {
    static final double W = 8.0518;
    static String suffix = "_mirrored";
    static final ObjectMapper mapper = new ObjectMapper().enable(SerializationFeature.INDENT_OUTPUT);

    public static void main(String[] args) throws Exception {
        if (args.length == 0) { System.out.println("Usage: MirrorPaths <file_or_dir> [suffix]"); return; }
        if (args.length > 1) suffix = args[1];

        Path target = Paths.get(args[0]);
        var files = Files.isRegularFile(target) ? java.util.List.of(target) :
            Files.walk(target).filter(p -> p.toString().matches(".*\\.(path|auto)") && !stem(p).endsWith(suffix)).collect(Collectors.toList());

        for (Path f : files) {
            JsonNode data = mapper.readTree(f.toFile());
            if (f.toString().endsWith(".path")) mirror((ObjectNode) data);
            Path out = f.resolveSibling(stem(f) + suffix + ext(f));
            mapper.writeValue(out.toFile(), data);
            System.out.println(f.getFileName() + " -> " + out.getFileName());
        }
    }

    static void mirror(ObjectNode d) {
        if (d.has("waypoints"))       for (JsonNode w : d.get("waypoints"))       mirrorWaypoint((ObjectNode) w);
        if (d.has("rotationTargets")) for (JsonNode r : d.get("rotationTargets")) ((ObjectNode)r).put("rotationDegrees", -r.get("rotationDegrees").asDouble());
        if (d.has("pointTowardsZones")) for (JsonNode z : d.get("pointTowardsZones")) mirrorPoint((ObjectNode) z.get("fieldPosition"));
        if (d.has("goalEndState"))      mirrorRot((ObjectNode) d.get("goalEndState"));
        if (d.has("idealStartingState")) mirrorRot((ObjectNode) d.get("idealStartingState"));
    }

    static void mirrorWaypoint(ObjectNode w) {
        mirrorPoint((ObjectNode) w.get("anchor"));
        if (!w.get("prevControl").isNull()) mirrorPoint((ObjectNode) w.get("prevControl"));
        if (!w.get("nextControl").isNull()) mirrorPoint((ObjectNode) w.get("nextControl"));
        if (w.has("heading") && !w.get("heading").isNull()) w.put("heading", -w.get("heading").asDouble());
    }

    static void mirrorPoint(ObjectNode p) { if (p != null) p.put("y", W - p.get("y").asDouble()); }
    static void mirrorRot(ObjectNode o)   { if (o != null && o.has("rotation")) o.put("rotation", -o.get("rotation").asDouble()); }

    static String stem(Path p) { String n = p.getFileName().toString(); return n.substring(0, n.lastIndexOf('.')); }
    static String ext(Path p)  { String n = p.getFileName().toString(); return n.substring(n.lastIndexOf('.')); }
}