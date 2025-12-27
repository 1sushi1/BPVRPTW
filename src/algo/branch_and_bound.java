package algo;
import java.util.ArrayList;
import java.io.IOException;
import java.util.PriorityQueue;
import java.util.Comparator;


public class branch_and_bound {
    double lowerbound; // 全局下界
    double upperbound; // 全局上界

    static class BranchFix {
        // 记录进行处理的边，以及是 forbid(0) 还是 impose(1)
        int from, to, val; // 0 forbid, 1 impose
        BranchFix(int f, int t, int v) { from = f; to = t; val = v; }
    }

    void printNodeHeader(BBNode node) {
        System.out.println(
                "\n[Node " + node.nodeId +
                        " | depth=" + node.depth +
                        " | fixes=" + node.fixes.size() +
                        "]"
        );
    }


    static class BBNode {
        // --- 树结构信息（你想要的）---
        int nodeId;         // 节点编号（方便追踪）
        BBNode parent;      // 指向父节点（可回溯整条分支路径）
        BranchFix lastFix;  // 父 -> 当前 这一步做的分支决定（根节点为 null）

        // --- 求解需要的信息 ---
        ArrayList<BranchFix> fixes;   // 从根到当前节点的全部分支决定（用于 applyFixes）
        ArrayList<route> routes;      // 传给 CG 的初始列
        int depth;                    // 深度
        double lowestValue;           // 该节点 LP 下界（CG obj），默认 +∞ 表示还没算

        BBNode(int nodeId, BBNode parent, BranchFix lastFix,
               ArrayList<BranchFix> fixes, ArrayList<route> routes, int depth) {
            this.nodeId = nodeId;
            this.parent = parent;
            this.lastFix = lastFix;

            this.fixes = fixes;
            this.routes = routes;
            this.depth = depth;

            this.lowestValue = Double.POSITIVE_INFINITY;
        }
    }

    static void applyFixes(paramsVRP p, ArrayList<BranchFix> fixes) {
        // 根据ArrayList生成这个节点的dist矩阵
        // reset dist
        for (int i = 0; i < p.nbclients + 2; i++) {
            System.arraycopy(p.distBase[i], 0, p.dist[i], 0, p.nbclients + 2);
        }

        // apply fixes in order
        for (BranchFix fx : fixes) {
            int u = fx.from, v = fx.to;

            if (fx.val == 0) {
                // forbid u->v
                p.dist[u][v] = p.verybig;
            } else {
                // impose u->v
                if (u != 0) {
                    for (int k = 0; k < p.nbclients + 2; k++) {
                        if (k != v) p.dist[u][k] = p.verybig;
                    }
                }
                if (v != p.nbclients + 1) {
                    for (int k = 0; k < p.nbclients + 2; k++) {
                        if (k != u) p.dist[k][v] = p.verybig;
                    }
                }
                // forbid opposite direction
                p.dist[v][u] = p.verybig;
            }
        }
    }

    boolean solveNodeByCG(paramsVRP p, BBNode node) throws IOException {
        // 调用CG求解当前节点的最优解
        applyFixes(p, node.fixes);

        columngen CG = new columngen();
        double CGobj = CG.computeColGen(p, node.routes);
        node.lowestValue = CGobj;

        // 你的兜底判定：异常就当 relax infeasible
        if ((CGobj > 2 * p.maxlength) || (CGobj < -1e-6)) {
            return false;
        }
        return true;
    }
    static class BranchChoice {
        boolean isIntegral;
        int u, v;
        int firstVal;  // 0 或 1：想先走哪个分支（dive时有用，PQ不强依赖）
    }

    static BranchChoice chooseBranchEdge(paramsVRP p, ArrayList<route> routes) {
        // 1) 由 routes.Q 汇总 edges
        for (int i = 0; i < p.nbclients + 2; i++) {
            java.util.Arrays.fill(p.edges[i], 0.0);
        }


        for (route r : routes) {
            if (r.getQ() > 1e-6) {
                int prev = 0;
                for (int k = 1; k < r.getpath().size(); k++) {
                    int city = r.getpath().get(k);
                    p.edges[prev][city] += r.getQ();
                    prev = city;
                }
            }
        }



        // 2) 找最分数边（用的 change = 0.5 - |coef-0.5|）
        boolean integral = true;
        double bestScore = -1;
        int bestU = -1, bestV = -1;
        int firstVal = 0;
        int cnt = 0;
        for (int i = 0; i < p.nbclients + 2; i++) {
            for (int j = 0; j < p.nbclients + 2; j++) {
                double coef = p.edges[i][j];
                if ((coef > 1e-6) && (coef < 0.9999999999 || coef > 1.0000000001)) {
                    integral = false;
//                    if (cnt < 10) {
//                        System.out.println("DEBUG FRACTIONAL EDGE FOUND: (" + i + "->" + j + ") = " + coef);
//                        cnt++;
//                    }
                    double score = 0.5 - Math.abs(coef - 0.5);
                    if (score > bestScore) {
                        bestScore = score;
                        bestU = i;
                        bestV = j;
                        // “先走更接近的那边”
                        firstVal = (Math.abs(1.0 - coef) > coef) ? 0 : 1;
                    }
                }
            }
        }

        BranchChoice bc = new BranchChoice();
        bc.isIntegral = integral;
        bc.u = bestU;
        bc.v = bestV;
        bc.firstVal = firstVal;
        return bc;
    }

    static ArrayList<route> filterRoutesForFix(paramsVRP p,
                                               ArrayList<route> parentRoutes,
                                               BranchFix fx) {
        // 为节点过滤掉不符合fx的route
        ArrayList<route> out = new ArrayList<>();
        int u = fx.from, v = fx.to;

        if (fx.val == 0) {
            // forbid: remove routes that contain arc u->v
            for (route r : parentRoutes) {
                ArrayList<Integer> path = r.getpath();
                boolean accept = true;
                if (path.size() > 3) { // keep trivial routes
                    int prev = 0;
                    for (int k = 1; accept && k < path.size(); k++) {
                        int city = path.get(k);
                        if (prev == u && city == v) accept = false;
                        prev = city;
                    }
                }
                if (accept) out.add(r);
            }
            return out;
        }

        // impose: keep only routes compatible with current dist
        for (route r : parentRoutes) {
            ArrayList<Integer> path = r.getpath();
            boolean accept = true;
            if (path.size() > 3) {
                int prev = 0;
                for (int k = 1; accept && k < path.size(); k++) {
                    int city = path.get(k);
                    if (p.dist[prev][city] >= p.verybig - 1e-6) accept = false;
                    prev = city;
                }
            }
            if (accept) out.add(r);
        }
        return out;
    }

    public boolean solveWithPQ(paramsVRP p,
                               ArrayList<route> rootRoutes,
                               ArrayList<route> bestRoutes) throws IOException {

        lowerbound = -1E10;
        upperbound = 1E10;

        int nodeIdGen = 0;

        PriorityQueue<BBNode> pq = new PriorityQueue<>(
                Comparator.comparingDouble(n -> n.lowestValue)
        );

        // 初始化root节点
        BBNode root = new BBNode(++nodeIdGen, null, null,
                new ArrayList<>(), rootRoutes, 0);

        if (solveNodeByCG(p, root)) {
            pq.add(root);
            lowerbound = pq.peek().lowestValue;
        } else {
            return false;
        }

        while (!pq.isEmpty()) {
            BBNode cur = pq.poll();

            printNodeHeader(cur);



            // 根据界限剪枝
            if (cur.lowestValue >= upperbound) {
                System.out.println(
                        "PRUNE BY BOUND | " +
                                " | nodeLB=" + cur.lowestValue +
                                " | globalLB=" + lowerbound +
                                " | globalUB=" + upperbound +
                                " | gap=" + ((upperbound - lowerbound) / upperbound)
                );
                continue;
            }

            // branching decision (based on cur.routes' Q)
            BranchChoice bc = chooseBranchEdge(p, cur.routes);


            // integral => incumbent
            if (bc.isIntegral) {
                if (cur.lowestValue < upperbound) {
                    upperbound = cur.lowestValue;
                    bestRoutes.clear();
                    for (route r : cur.routes) {
                        if (r.getQ() > 1e-6) {
                            route rr = new route();
                            rr.setcost(r.getcost());
                            rr.path = r.getpath();
                            rr.setQ(r.getQ());
                            bestRoutes.add(rr);
                        }

                    }
                    System.out.println("INCUMBENT | nodeLB="+ cur.lowestValue + " | globalLB="+ lowerbound + " | globalUB=" + upperbound
                            + " | gap=" + ((upperbound - lowerbound) / upperbound)
                            );
                }
                continue;
            }

            // --- generate two children ---

            System.out.println(
                    "FRACTIONAL SOLUTION" +
                            " | nodeLB=" + cur.lowestValue +
                            " | globalLB=" + lowerbound +
                            " | globalUB=" + upperbound +
                            " | gap=" + ((upperbound - lowerbound) / upperbound)
                    + "\n → FRACTIONAL SOLUTION, branch on edge (" +
                            bc.u + " -> " + bc.v + " )"
            );
            int valA = bc.firstVal;        // 第一个分支（dive方向）
            int valB = 1 - bc.firstVal;    // 第二个分支

            // child A: firstVal
            BranchFix fxA = new BranchFix(bc.u, bc.v, valA);
            ArrayList<BranchFix> fixesA = new ArrayList<>(cur.fixes);
            fixesA.add(fxA);

            BBNode childA = new BBNode(++nodeIdGen, cur, fxA, fixesA, null, cur.depth + 1);
            applyFixes(p, childA.fixes);
            childA.routes = filterRoutesForFix(p, cur.routes, fxA);
            boolean solvedA = solveNodeByCG(p, childA);
            if (!solvedA) {
                System.out.println(
                        "PRUNE RELAX INFEASIBLE | NodeId=" + childA.nodeId +
                                " | depth=" + childA.depth +
                                " | routes=" + childA.routes.size()
                );
            } else if (childA.lowestValue >= upperbound) {
                System.out.println(
                        "PRUNE BY BOUND | NodeId=" + childA.nodeId +
                                " | nodeLB=" + childA.lowestValue +
                                " | globalUB=" + upperbound +
                                " | globalLB=" + lowerbound +
                                " | gap=" + ((upperbound - lowerbound) / upperbound) +
                                " | depth=" + childA.depth +
                                " | routes=" + childA.routes.size()
                );
            } else {
                pq.add(childA);
                System.out.println(
                        "ENQUEUE | NodeId=" + childA.nodeId +
                                " | fix=(" + bc.u + "->" + bc.v + ")=" + valA +
                                " | nodeLB=" + childA.lowestValue +
                                " | depth=" + childA.depth +
                                " | routes=" + childA.routes.size()
                );
            }
            // child B: 1-firstVal
            BranchFix fxB = new BranchFix(bc.u, bc.v, valB);
            ArrayList<BranchFix> fixesB = new ArrayList<>(cur.fixes);
            fixesB.add(fxB);

            BBNode childB = new BBNode(++nodeIdGen, cur, fxB, fixesB, null, cur.depth + 1);
            applyFixes(p, childB.fixes);
            childB.routes = filterRoutesForFix(p, cur.routes, fxB);
            boolean solvedB = solveNodeByCG(p, childB);
            if (!solvedB) {
                System.out.println(
                        "PRUNE RELAX INFEASIBLE | NodeId=" + childB.nodeId +
                                " | depth=" + childB.depth +
                                " | routes=" + childB.routes.size()
                );
            } else if (childB.lowestValue >= upperbound) {
                System.out.println(
                        "PRUNE BY BOUND | NodeId=" + childB.nodeId +
                                " | nodeLB=" + childB.lowestValue +
                                " | globalUB=" + upperbound +
                                " | globalLB=" + lowerbound +
                                " | gap=" + ((upperbound - lowerbound) / upperbound) +
                                " | depth=" + childB.depth +
                                " | routes=" + childB.routes.size()
                );
            } else {
                pq.add(childB);
                System.out.println(
                        "ENQUEUE | NodeId=" + childB.nodeId +
                                " | fix=(" + bc.u + "->" + bc.v + ")=" + valB +
                                " | nodeLB=" + childB.lowestValue +
                                " | depth=" + childB.depth +
                                " | routes=" + childB.routes.size()
                );
            }

            // strict globalLB
            if (!pq.isEmpty()) lowerbound = pq.peek().lowestValue;

            // gap stop
            if (upperbound < 1e9 && (upperbound - lowerbound) / upperbound < p.gap) {
                System.out.println("STOP | LB=" + lowerbound + " UB=" + upperbound);
                return true;
            }
        }

        return true;
    }









}
