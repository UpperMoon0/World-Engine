import jdk.jfr.consumer.RecordingFile;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.regex.Pattern;

/** java tools/JfrHotspots.java path/to/profile.jfr (Java 21, no extra dependencies). */
public class JfrHotspots {
    private static long epoch(String sample, String field, long fallback) {
        var match = Pattern.compile("\"" + field + "\"\\s*:\\s*(\\d+)").matcher(sample);
        return match.find() ? Long.parseLong(match.group(1)) : fallback;
    }

    public static void main(String[] args) throws Exception {
        if (args.length != 1) throw new IllegalArgumentException("Pass exactly one profile.jfr path");
        Path profile = Path.of(args[0]);
        Path samplePath = profile.resolveSibling("sample.json");
        String sample = Files.exists(samplePath) ? Files.readString(samplePath) : "";
        long start = epoch(sample, "measurementStartEpochMs", Long.MIN_VALUE);
        long end = epoch(sample, "measurementEndEpochMs", Long.MAX_VALUE);
        System.out.println("Profile: " + profile.toAbsolutePath());
        System.out.println(start == Long.MIN_VALUE ? "Scope: entire server lifecycle (no measurement timestamps)"
                : "Scope: measured window " + start + " .. " + end + " epoch ms");
        System.out.println("Allocation weights are sampled estimates; CPU values are sample counts.");
        Map<String, Long> totals = new HashMap<>();
        List<String> types = List.of("jdk.ObjectAllocationSample", "jdk.ExecutionSample", "jdk.NativeMethodSample");
        try (var file = new RecordingFile(profile)) {
            while (file.hasMoreEvents()) {
                var event = file.readEvent();
                String type = event.getEventType().getName();
                if (!types.contains(type)) continue;
                long when = event.getStartTime().toEpochMilli();
                if (when < start || when > end) continue;
                var thread = event.getThread(type.equals("jdk.ObjectAllocationSample") ? "eventThread" : "sampledThread");
                if (thread == null || !(thread.getJavaName().equals("Server thread")
                        || thread.getJavaName().equals("Sable-Rapier-Region"))) continue;
                var trace = event.getStackTrace();
                if (trace == null || trace.getFrames().isEmpty()) continue;
                var frames = trace.getFrames();
                String owner = null;
                for (var frame : frames) {
                    String name = frame.getMethod().getType().getName();
                    if (name.startsWith("com.nstut.worldengine") || name.startsWith("dev.ryanhcode.sable")) {
                        owner = name + "." + frame.getMethod().getName() + ":" + frame.getLineNumber();
                        break;
                    }
                }
                if (owner == null) continue;
                long value = type.equals("jdk.ObjectAllocationSample") ? event.getLong("weight") : 1;
                var method = frames.getFirst().getMethod();
                String leaf = method.getType().getName() + "." + method.getName();
                totals.merge(type + " | " + owner + " | " + leaf, value, Long::sum);
            }
        }
        for (String type : types) {
            System.out.println(type);
            totals.entrySet().stream().filter(row -> row.getKey().startsWith(type))
                    .sorted(Map.Entry.<String, Long>comparingByValue().reversed()).limit(20)
                    .forEach(row -> System.out.println(row.getValue() + " " + row.getKey()));
        }
    }
}
