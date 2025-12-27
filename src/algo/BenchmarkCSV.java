package algo;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;



public class BenchmarkCSV {

    // -----------------------------
    // CSV 工具：处理逗号、换行、引号
    // -----------------------------
    private static class CSVUtil {
        static void writeLine(BufferedWriter bw, String... cols) throws IOException {
            StringBuilder sb = new StringBuilder();
            for (int i = 0; i < cols.length; i++) {
                if (i > 0) sb.append(",");
                sb.append(escape(cols[i]));
            }
            bw.write(sb.toString());
            bw.newLine();
        }

        private static String escape(String s) {
            if (s == null) return "";
            boolean needQuote = s.contains(",") || s.contains("\n") || s.contains("\r") || s.contains("\"");
            s = s.replace("\"", "\"\"");
            return needQuote ? "\"" + s + "\"" : s;
        }
    }

    // -----------------------------
    // 结果结构
    // -----------------------------
    private static class RunResult {
        final String dataset;
        final String algo;
        final double timeSec;
        final double bestCost;
        final ArrayList<route> bestRoutes;
        final String status;   // OK / ERROR
        final String errorMsg; // exception string if ERROR

        RunResult(String dataset, String algo, double timeSec, double bestCost,
                  ArrayList<route> bestRoutes, String status, String errorMsg) {
            this.dataset = dataset;
            this.algo = algo;
            this.timeSec = timeSec;
            this.bestCost = bestCost;
            this.bestRoutes = bestRoutes;
            this.status = status;
            this.errorMsg = errorMsg;
        }
    }

    private static double computeCost(ArrayList<route> bestRoutes) {
        double optCost = 0.0;
        for (route r : bestRoutes) optCost += r.cost;
        return optCost;
    }

    // -----------------------------
    // 跑 PQ：branch_and_bound.solveWithPQ
    // -----------------------------
    private static RunResult runPQ(String datasetPath) {
        final String algo = "PQ(solveWithPQ)";
        try {
            paramsVRP instance = new paramsVRP();
            instance.initParams(datasetPath);

            branch_and_bound bp = new branch_and_bound();
            ArrayList<route> rootRoutes = new ArrayList<>();
            ArrayList<route> bestRoutes = new ArrayList<>();

            long t0 = System.nanoTime();
            bp.solveWithPQ(instance, rootRoutes, bestRoutes);
            long t1 = System.nanoTime();

            double sec = (t1 - t0) / 1e9;
            double cost = computeCost(bestRoutes);

            return new RunResult(datasetPath, algo, sec, cost, bestRoutes, "OK", "");
        } catch (Exception e) {
            return new RunResult(datasetPath, algo, Double.NaN, Double.NaN,
                    new ArrayList<>(), "ERROR", e.toString());
        }
    }

    // -----------------------------
    // 跑递归：branchandbound.BBNode
    // -----------------------------
    private static RunResult runRecursive(String datasetPath) {
        final String algo = "REC(BBNode)";
        try {
            paramsVRP instance = new paramsVRP();

            instance.initParams(datasetPath);

            branchandbound bp = new branchandbound();
            ArrayList<route> initRoutes = new ArrayList<>();
            ArrayList<route> bestRoutes = new ArrayList<>();

            long t0 = System.nanoTime();
            bp.BBNode(instance, initRoutes, null, bestRoutes, 0);
            long t1 = System.nanoTime();

            double sec = (t1 - t0) / 1e9;
            double cost = computeCost(bestRoutes);

            return new RunResult(datasetPath, algo, sec, cost, bestRoutes, "OK", "");
        } catch (Exception e) {
            return new RunResult(datasetPath, algo, Double.NaN, Double.NaN,
                    new ArrayList<>(), "ERROR", e.toString());
        }
    }

    // -----------------------------
    // 构造数据集列表
    // 注意：你的文件名如果是 .txt 小写，请改这里
    // -----------------------------
    private static List<String> buildDatasets() {
        List<String> ds = new ArrayList<>();
        for (int i = 104; i <= 109; i++) ds.add("dataset/c" + i + ".TXT");
        for (int i = 201; i <= 208; i++) ds.add("dataset/c" + i + ".TXT");
        return ds;
    }

    // -----------------------------
    // 把结果写到 CSV：summary + routes
    // -----------------------------
    private static void appendSummary(BufferedWriter bw, RunResult res) throws IOException {
        CSVUtil.writeLine(
                bw,
                res.dataset,
                res.algo,
                res.status,
                Double.isNaN(res.timeSec) ? "" : String.format("%.6f", res.timeSec),
                Double.isNaN(res.bestCost) ? "" : String.valueOf(res.bestCost),
                String.valueOf(res.bestRoutes.size()),
                res.errorMsg == null ? "" : res.errorMsg
        );
    }

    private static void appendRoutes(BufferedWriter bw, RunResult res) throws IOException {
        int rid = 1;
        for (route r : res.bestRoutes) {
            CSVUtil.writeLine(
                    bw,
                    res.dataset,
                    res.algo,
                    String.valueOf(rid++),
                    String.valueOf(r.cost),
                    String.valueOf(r.path)
            );
        }
        // 如果 ERROR 或者没解出来，也留个记录（可选）
        if (!"OK".equals(res.status) || res.bestRoutes.isEmpty()) {
            CSVUtil.writeLine(
                    bw,
                    res.dataset,
                    res.algo,
                    "0",
                    "",
                    ("OK".equals(res.status) ? "NO_ROUTES" : ("ERROR: " + res.errorMsg))
            );
        }
    }

    public static void main(String[] args) throws IOException {
        List<String> datasets = buildDatasets();
        System.out.println("Max heap (MB) = " +
                Runtime.getRuntime().maxMemory() / 1024 / 1024);

        // 输出文件
        String summaryPath = "summary.csv";
        String routesPath  = "routes.csv";

        // 打开 writer
        try (BufferedWriter summary = new BufferedWriter(new FileWriter(summaryPath));
             BufferedWriter routes  = new BufferedWriter(new FileWriter(routesPath))) {

            // 写表头
            CSVUtil.writeLine(summary, "Dataset", "Algorithm", "Status", "TimeSec", "BestCost", "NumRoutes", "Error");
            CSVUtil.writeLine(routes, "Dataset", "Algorithm", "RouteId", "RouteCost", "Path");

            // 逐个数据集运行
            for (String ds : datasets) {
                System.out.println("Running dataset: " + ds);

                // PQ
                System.gc(); // 可选：减少相互影响（不保证）
                RunResult pq = runPQ(ds);
                appendSummary(summary, pq);
                appendRoutes(routes, pq);
                summary.flush();
                routes.flush();
                System.out.println("  PQ  => " + pq.status + ", time=" + pq.timeSec + ", cost=" + pq.bestCost);

                // REC
                System.gc();
                RunResult rec = runRecursive(ds);
                appendSummary(summary, rec);
                appendRoutes(routes, rec);
                summary.flush();
                routes.flush();
                System.out.println("  REC => " + rec.status + ", time=" + rec.timeSec + ", cost=" + rec.bestCost);
            }
        }

        System.out.println("\nDone.");
        System.out.println("Saved: " + summaryPath);
        System.out.println("Saved: " + routesPath);
    }
}
