package algo;

/**
 * Asymmetric VRP with Resources Constraints (Time Windows and Capacity)
 * Branch and Price algorithm (Branch and Bound + Column generation)
 * For educational purpose only! No code optimization.
 * * Converted from CPLEX to Gurobi Optimizer.
 */

import java.io.IOException;
import java.text.DecimalFormat;
import java.util.ArrayList;

// 引入 Gurobi 包
import com.gurobi.gurobi.*;

public class columngen {

	// 辅助类：用于存储 Gurobi 变量，模仿原代码中的 IloNumVarArray
	static class GRBVarArray {
		int _num = 0;
		GRBVar[] _array = new GRBVar[32];

		void add(GRBVar ivar) {
			if (_num >= _array.length) {
				GRBVar[] array = new GRBVar[2 * _array.length];
				System.arraycopy(_array, 0, array, 0, _num);
				_array = array;
			}
			_array[_num++] = ivar;
		}

		GRBVar getElement(int i) {
			return _array[i];
		}

		int getSize() {
			return _num;
		}
	}

	public double computeColGen(paramsVRP userParam, ArrayList<route> routes)
			throws IOException {
		int i, j, prevcity, city;
		double cost, obj;
		double[] pi;
		boolean oncemore;

		try {

			// ---------------------------------------------------------
			// 1. 初始化 Gurobi 环境和模型 (Restricted Master Problem)
			// ---------------------------------------------------------
			// 创建环境 (Empty env allows setting params before start)
			GRBEnv env = new GRBEnv(true);
			env.set(GRB.IntParam.OutputFlag, 0); // 禁止输出 Gurobi 日志
			env.start();

			GRBModel model = new GRBModel(env);

			// 设置为最小化问题
			model.set(GRB.IntAttr.ModelSense, GRB.MINIMIZE);

			// 设置算法为原初单纯形法 (Primal Simplex)，对应原代码的 RootAlgorithm.Primal
			model.set(GRB.IntParam.Method, GRB.METHOD_PRIMAL);

			// ---------------------------------------------------------
			// 2. 定义约束 (Constraints)
			// ---------------------------------------------------------
			// 对应原代码：lpmatrix[i] = cplex.addRange(1.0, Double.MAX_VALUE);
			// 含义：每个客户至少被访问一次 (sum(y_p) >= 1)
			// 我们先创建线性约束，右端项为 1.0，符号为 >= (GRB.GREATER_EQUAL)
			// 注意：初始时 LHS 为空，稍后通过 Column 添加变量

			GRBConstr[] lpmatrix = new GRBConstr[userParam.nbclients];
			for (i = 0; i < userParam.nbclients; i++) {
				// addConstr(lhs, sense, rhs, name)
				lpmatrix[i] = model.addConstr(new GRBLinExpr(), GRB.GREATER_EQUAL, 1.0, "c" + i);
			}

			// 变量容器
			GRBVarArray y = new GRBVarArray();

			// ---------------------------------------------------------
			// 3. 初始化变量 (Variables / Columns)
			// ---------------------------------------------------------
			// 将现有的 routes 添加到模型中
			for (route r : routes) {
				int v;
				cost = 0.0;
				prevcity = 0;
				for (i = 1; i < r.getpath().size(); i++) {
					city = r.getpath().get(i);
					cost += userParam.dist[prevcity][city];
					prevcity = city;
				}

				r.setcost(cost);

				// --- 创建 Gurobi Column 对象 ---
				// Gurobi 的 Column 用于指定新变量在哪些约束中系数不为 0
				GRBColumn column = new GRBColumn();

				// 目标函数系数在 addVar 中直接指定，这里只处理约束系数
				for (i = 1; i < r.getpath().size() - 1; i++) {
					v = r.getpath().get(i) - 1;
					// 向 Column 添加项：系数 1.0，对应约束 lpmatrix[v]
					column.addTerm(1.0, lpmatrix[v]);
				}

				// 添加变量: addVar(lb, ub, obj, vtype, column, name)
				// lb=0.0, ub=INFINITY, obj=cost, type=CONTINUOUS
				y.add(model.addVar(0.0, GRB.INFINITY, r.getcost(), GRB.CONTINUOUS, column, "y_init_" + y.getSize()));
			}

			// 如果初始路径不足以覆盖所有客户，添加简单的往返路径 (Depot -> Client -> Depot)
			if (routes.size() < userParam.nbclients) {
				for (i = 0; i < userParam.nbclients; i++) {
					cost = userParam.dist[0][i + 1]
							+ userParam.dist[i + 1][userParam.nbclients + 1];

					GRBColumn column = new GRBColumn();
					column.addTerm(1.0, lpmatrix[i]); // 对应第 i 个客户的约束

					y.add(model.addVar(0.0, GRB.INFINITY, cost, GRB.CONTINUOUS, column, "y_dummy_" + i));

					route newroute = new route();
					newroute.addcity(0);
					newroute.addcity(i + 1);
					newroute.addcity(userParam.nbclients + 1);
					newroute.setcost(cost);
					routes.add(newroute);
				}
			}

			// ---------------------------------------------------------
			// 4. 列生成循环 (Column Generation Loop)
			// ---------------------------------------------------------
			DecimalFormat df = new DecimalFormat("#0000.00");
			oncemore = true;
			double[] prevobj = new double[100];
			int nbroute;
			int previ = -1;

			// Gurobi 在循环中添加变量推荐 update，但在 optimize 前会自动处理，此处显式保留结构

			while (oncemore) {
				oncemore = false;

				// --- 求解 RMP ---
				model.optimize();

				// 检查求解状态
				int status = model.get(GRB.IntAttr.Status);
				if (status == GRB.Status.INF_OR_UNBD || status == GRB.Status.INFEASIBLE || status == GRB.Status.UNBOUNDED) {
					System.out.println("CG: relaxation infeasible!");
					model.dispose();
					env.dispose();
					return 1E10;
				}

				prevobj[(++previ) % 100] = model.get(GRB.DoubleAttr.ObjVal);

				// ---------------------------------------------------------
				// 获取对偶值 (Dual Values / Pi)
				// ---------------------------------------------------------
				pi = new double[userParam.nbclients];
				for (i = 0; i < userParam.nbclients; i++) {
					pi[i] = lpmatrix[i].get(GRB.DoubleAttr.Pi);
				}

				// --- 更新子问题 (SPPRC) 的边权重 ---
				// Reduced Cost C_ij = Dist_ij - Dual_i
				// 注意：Dual_i 对应客户 i 的约束。
				// 逻辑：cost[i][j] = dist[i][j] - pi[i-1]
				for (i = 1; i < userParam.nbclients + 1; i++)
					for (j = 0; j < userParam.nbclients + 2; j++)
						userParam.cost[i][j] = userParam.dist[i][j] - pi[i - 1];

				// --- 求解子问题 (SPPRC) ---
				SPPRC sp = new SPPRC();
				ArrayList<route> routesSPPRC = new ArrayList<route>();

				nbroute = userParam.nbclients;
				// 原代码中有关于收敛速度的注释逻辑，此处保留结构

				sp.shortestPath(userParam, routesSPPRC, nbroute);
				sp = null;

				// --- 检查是否找到负 Reduced Cost 的列 ---
				if (routesSPPRC.size() > 0) {
					for (route r : routesSPPRC) {
						ArrayList<Integer> rout = r.getpath();
						prevcity = rout.get(1);

						// 计算真实成本 (Real Cost) 用于 RMP 的目标函数系数
						cost = userParam.dist[0][prevcity];

						// 创建 Gurobi Column
						GRBColumn column = new GRBColumn();
						column.addTerm(1.0, lpmatrix[rout.get(1) - 1]);

						for (i = 2; i < rout.size() - 1; i++) {
							city = rout.get(i);
							cost += userParam.dist[prevcity][city];
							prevcity = city;

							// 添加约束系数
							column.addTerm(1.0, lpmatrix[rout.get(i) - 1]);
						}
						cost += userParam.dist[prevcity][userParam.nbclients + 1];

						// 向模型添加新变量 (Column)
						y.add(model.addVar(0.0, GRB.INFINITY, cost, GRB.CONTINUOUS, column, "P" + routes.size()));

						r.setcost(cost);
						routes.add(r);

						oncemore = true;
					}
					System.out.print("\nCG Iter " + previ + " Current cost: "
							+ df.format(prevobj[previ % 100]) + " " + routes.size()
							+ " routes");
					System.out.flush();
				}

				// 显式清理
				routesSPPRC = null;
			}

			System.out.println();

			// 更新路由的流量值 (q)
			for (i = 0; i < y.getSize(); i++) {
				routes.get(i).setQ(y.getElement(i).get(GRB.DoubleAttr.X));
			}

			obj = model.get(GRB.DoubleAttr.ObjVal);

			// 清理 Gurobi 资源
			model.dispose();
			env.dispose();

			return obj;

		} catch (GRBException e) {
			System.err.println("Gurobi exception caught: Code " + e.getErrorCode() + " - " + e.getMessage());
		}
		return 1E10;
	}
}