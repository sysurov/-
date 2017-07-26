﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using xna = Microsoft.Xna.Framework;
using URWPGSim2D.Common;
using URWPGSim2D.StrategyLoader;
using URWPGSim2D.StrategyHelper;
using System.IO;

namespace URWPGSim2D.Strategy {
	public class Strategy : MarshalByRefObject, IStrategy {
		#region reserved code never be changed or removed
		/// <summary>
		/// override the InitializeLifetimeService to return null instead of a valid ILease implementation
		/// to ensure this type of remote object never dies
		/// </summary>
		/// <returns>null</returns>
		public override object InitializeLifetimeService() {
			//return base.InitializeLifetimeService();
			return null; // makes the object live indefinitely
		}
		#endregion

		/// <summary>
		/// 决策类当前对象对应的仿真使命参与队伍的决策数组引用 第一次调用GetDecision时分配空间
		/// </summary>
		private Decision[] decisions = null;

		public static int completeCircle = 0;
		Decision[] preDecisions = null;

		// 表演阶段标记
		private static int stage = 2;

		private static int[] timeForPoseToPose = new int[11];
		private static bool completeFlag = false;

		/// <summary>
		/// 获取队伍名称 在此处设置参赛队伍的名称
		/// </summary>
		/// <returns>队伍名称字符串</returns>
		public string GetTeamName() {
			return "SYSU East Repairers";
		}

		public static int isDirectionRight(float a, float b) {
			if (a > Math.PI)
				a -= (float)(2 * Math.PI);
			if (a < -Math.PI)
				a += (float)(2 * Math.PI);

			if (b > Math.PI)
				b -= (float)(2 * Math.PI);
			if (b < -Math.PI)
				b += (float)(2 * Math.PI);

			if (a - b > 0.2)
				return 1;
			else if (a - b < -0.2)
				return -1;
			else
				return 0;
		}

		public static bool allEqual(int[] group, int value, int start, int end) {
			for (int i = start; i <= end; i++)
				if (group[i] != value)
					return false;
			return true;
		}

		// 判断group数组内从start到end间是否有不少于numToPass数量的项等于value
		public static bool isNearlyStable(int[] group, int value, int start, int end, int numToPass) {
			int counter = 0;
			for (int i = start; i <= end; i++)
				if (group[i] == value)
					counter++;
			return (counter >= numToPass) ? true : false;
		}

		// 
		public static bool allFishNearPoints(RoboFish[] group, xna.Vector3[] target, int start, int end, float aRadius) {
			for (int i = start; i <= end; i++)
				if ((Math.Pow(group[i].PositionMm.X - target[i].X, 2) + Math.Pow(group[i].PositionMm.Z - target[i].Z, 2)) > Math.Pow(aRadius, 2))
					return false;
			return true;
		}

		public static void stopFish(ref Decision decision, int i) {
			decision.VCode = 0;
			decision.TCode = 7;
			timeForPoseToPose[i] = 0;
		}

		// 获得两个坐标间的距离
		public static float getVectorDistance(xna.Vector3 a, xna.Vector3 b) {
			return (float)Math.Sqrt((Math.Pow((a.X - b.X), 2d) + Math.Pow((a.Z - b.Z), 2d)));
		}

		// 角度转弧度，为了一定的可读性
		private static float deg2rad(float deg) {
			double rad = (Math.PI * deg / 180);
			if (rad > Math.PI)
				rad -= (float)(2 * Math.PI);
			if (rad < -Math.PI)
				rad += (float)(2 * Math.PI);

			return (float)rad;
		}

		public static void fish2Point(ref Decision decisions, RoboFish fish, xna.Vector3 targetPoint, float targetDirection,
										int fishId, ref int[] timeForPoseToPose, int[] flag) {
			switch (flag[fishId]) {
				case 0:
					// 距离目标点超过100，调用p2p函数，当处于目标点附近100mm的范围内时，进入下一个状态
					if (getVectorDistance(targetPoint, fish.PositionMm) > 100)
						Helpers.PoseToPose(ref decisions, fish, targetPoint, targetDirection, 6f, 10f, 100, ref timeForPoseToPose[fishId]);
					if (getVectorDistance(targetPoint, fish.PositionMm) < 100)
						flag[fishId] = 1;
					break;
				case 1:
					// 角度调整至期望的角度，此时因为速度值不能为0，鱼可能会继续前进，因此将距离范围阈值调整至150
					// 当角度处于期望的范围内，进入下个状态，如果滑出了距离范围，重新开始整个过程
					if (isDirectionRight(targetDirection, fish.BodyDirectionRad) < 0) {
						decisions.TCode = 0;
						decisions.VCode = 1;
					} else if (isDirectionRight(targetDirection, fish.BodyDirectionRad) > 0) {
						decisions.TCode = 14;
						decisions.VCode = 1;
					} else {
						flag[fishId] = 2;
						stopFish(ref decisions, fishId);
					}
					if (getVectorDistance(targetPoint, fish.PositionMm) > 150)
						flag[fishId] = 0;
					break;
				case 2:
					// 重复上一个阶段，如果ok，结束
					if (isDirectionRight(targetDirection, fish.BodyDirectionRad) < 0) {
						decisions.TCode = 0;
						decisions.VCode = 1;
					} else if (isDirectionRight(targetDirection, fish.BodyDirectionRad) > 0) {
						decisions.TCode = 14;
						decisions.VCode = 1;
					} else
						stopFish(ref decisions, fishId);
					if (getVectorDistance(targetPoint, fish.PositionMm) > 150)
						flag[fishId] = 0;
					break;
				default:
					// 反常状态
					stopFish(ref decisions, fishId);
					break;
			}
		}

		public static void fish2Point_Fast(ref Decision decisions, RoboFish fish, xna.Vector3 targetPoint, float targetDirection,
											int fishId, ref int[] timeForPoseToPose, int[] flag) {
			switch (flag[fishId]) {
				case 0:
					// 带球的dribble更快一些
					if (getVectorDistance(targetPoint, fish.PositionMm) > 200)
						Helpers.Dribble(ref decisions, fish, targetPoint, targetDirection, 20f, 30f, 200, 14, 12, 15, 100, true);
					else if (getVectorDistance(targetPoint, fish.PositionMm) > 180)
						Helpers.PoseToPose(ref decisions, fish, targetPoint, targetDirection, 45f, 50f, 100, ref timeForPoseToPose[fishId]);
					else if (getVectorDistance(targetPoint, fish.PositionMm) <= 180)
						flag[fishId] = 1;
					break;
				case 1:
					// 与原类似
					if (getVectorDistance(targetPoint, fish.PositionMm) > 250)
						flag[fishId] = 0;
					else if (isDirectionRight(targetDirection, fish.BodyDirectionRad) < 0) {
						decisions.TCode = 0;
						decisions.VCode = 1;
					} else if (isDirectionRight(targetDirection, fish.BodyDirectionRad) > 0) {
						decisions.TCode = 14;
						decisions.VCode = 1;
					} else {
						flag[fishId] = 2;
						stopFish(ref decisions, fishId);
					}
					break;
				case 2:
					// ？？？，比原方案增加了一些调整
					if (getVectorDistance(targetPoint, fish.PositionMm) > 250)
						flag[fishId] = 0;
					else if (isDirectionRight(targetDirection, fish.BodyDirectionRad) != 0)
						flag[fishId] = 1;
					else
						stopFish(ref decisions, fishId);
					break;
				default:
					stopFish(ref decisions, fishId);
					break;
			}
		}

		public static bool isInSector(xna.Vector3 point, xna.Vector3 targetPoint,
										int sectorR, float sectorTheta, float sectorBaseAngle) {
			if (Math.Pow(targetPoint.X - point.X, 2) + Math.Pow(targetPoint.Z - point.Z, 2) > Math.Pow(sectorR, 2))
				// 点在整圆外
				return false;
			else {
				// 首先求和扇形中线的夹角
				double theta = (sectorBaseAngle - (Math.Atan((float)(targetPoint.X - point.X) / (float)(targetPoint.Z - point.Z))));
				if (Math.Abs(theta) < sectorTheta)
					return true;
				else
					return false;
			}
		}

		public static int isDirectionInRange(float targetDir, float dir, float dirRange) {
			if (targetDir > Math.PI)
				targetDir -= (float)(2 * Math.PI);
			if (targetDir < -Math.PI)
				targetDir += (float)(2 * Math.PI);

			if (dir > Math.PI)
				dir -= (float)(2 * Math.PI);
			if (dir < -Math.PI)
				dir += (float)(2 * Math.PI);

			if (targetDir - dir > dirRange)
				return 1;
			else if (targetDir - dir < -dirRange)
				return -1;
			else
				return 0;
		}

		// 只要在目标点前方（由targetDir确定）圆心角2*tSectorTheta，半径tSectorR的扇形区域内，角度与targetDir的偏差在±tTheta内都认定为已经到达目标点
		public static void fish2Point_Forward(ref Decision decisions, RoboFish fish, xna.Vector3 targetPoint, float targetDirection,
												int fishId, ref int[] timeForPoseToPose, int[] flag,
												int tSectorR, float tSectorTheta, float tTheta, int tDistance) {
			switch (flag[fishId]) {
				case 0:
					if (getVectorDistance(targetPoint, fish.PositionMm) > tDistance && !isInSector(fish.PositionMm, targetPoint, tSectorR, tSectorTheta, targetDirection))
						Helpers.PoseToPose(ref decisions, fish, targetPoint, targetDirection, 6f, 10f, 100, ref timeForPoseToPose[fishId]);
					if (getVectorDistance(targetPoint, fish.PositionMm) < tDistance || isInSector(fish.PositionMm, targetPoint, tSectorR, tSectorR, targetDirection))
						flag[fishId] = 1;
					break;
				case 1:
					if (getVectorDistance(targetPoint, fish.PositionMm) > tDistance && !isInSector(fish.PositionMm, targetPoint, tSectorR, tSectorTheta, targetDirection))
						flag[fishId] = 0;
					else if (isDirectionInRange(targetDirection, fish.BodyDirectionRad, tTheta) < 0) {
						decisions.TCode = 0;
						decisions.VCode = 1;
					} else if (isDirectionInRange(targetDirection, fish.BodyDirectionRad, tTheta) > 0) {
						decisions.TCode = 14;
						decisions.VCode = 1;
					} else {
						flag[fishId] = 2;
						stopFish(ref decisions, fishId);
					}
					break;
				case 2:
					if (getVectorDistance(targetPoint, fish.PositionMm) > (tDistance * 3 / 2) && !isInSector(fish.PositionMm, targetPoint, tSectorR, tSectorTheta, targetDirection))
						flag[fishId] = 0;
					else if (isDirectionInRange(targetDirection, fish.BodyDirectionRad, tTheta) != 0)
						flag[fishId] = 1;
					else
						stopFish(ref decisions, fishId);
					break;

				default:
					// 反常状态
					stopFish(ref decisions, fishId);
					break;
					// 使用poseToPose进行运动
			}
		}

		// 倾向于走弧线的运动算法，预期运动半径为arcR
		public static void fish2Point_Arc(ref Decision decisons, RoboFish fish, xna.Vector3 targetPoint, float targetAngle,
											int fishId, ref int[] timeForPoseToPose, int[] flag,
											int arcR) {

		}
		#region NumberTen
		//阿拉伯数字造型10
		//程钰
		//v0.2 07.24:加入最近算法
		// 07.25 加入超时和切换功能
		//运气不好的话还是会一直转圈
		//我感觉fish2Point可以修改一下？让他不要那么蠢总是转转转
		//算法还没debug，肉眼debug没啥问题= =
		class NumberTenClass {
			private static int state = 0;
			// 计时器与超时
			private static long timer = 0;
			private static long timingLimit;
			private static int endingDelay;

			static xna.Vector3[] targetVector = new xna.Vector3[9];
			static float[] targetAngle = new float[9];
			// 稳定标记
			private static int[] eqFlag = new int[11];
			private static bool[] DesUse = { false, false, false, false, false, false, false, false, false };
			private static bool[] FishUse = { false, false, false, false, false, false, false, false, false };
			private static int[] FishDes = new int[9];  //鱼的目标点

			static int[] targetVectorX1 = { -700, -700, -700, 0, 290, 670, 670, 470, 50 };
			static int[] targetVectorZ1 = { -480, 200, 480, 120, 670, 300, -270, -670, -350 };
			static float[] targetAngle1 = { 90, 90, 270, 90, 43, 300, 270, 230, 120 };

			private static float deg2rad(float deg) {
				if (deg > 180) deg -= 360;
				return (float)(Math.PI * deg / 180);
			}


			public static void movingTen(ref Mission mission, int teamId, ref Decision[] decisions) {
				// 仿真周期毫秒数
				int msPerCycle = mission.CommonPara.MsPerCycle;
				RoboFish[] fish = {
					mission.TeamsRef[teamId].Fishes[1],
					mission.TeamsRef[teamId].Fishes[2],
					mission.TeamsRef[teamId].Fishes[3],
					mission.TeamsRef[teamId].Fishes[4],
					mission.TeamsRef[teamId].Fishes[5],
					mission.TeamsRef[teamId].Fishes[6],
					mission.TeamsRef[teamId].Fishes[7],
					mission.TeamsRef[teamId].Fishes[8],
					mission.TeamsRef[teamId].Fishes[9] };

				float[,] distance = new float[9, 9];
				// 鱼到目标点的距离矩阵
				if (state == 0) {
					// 初始化
					state = 1;
					for (int i = 0; i < 11; i++) {
						eqFlag[i] = 0;
						timeForPoseToPose[i] = 0;
					}

					// 时限1分钟
					timingLimit = (2) * 30 * 1000 / mission.CommonPara.MsPerCycle;
					// 结尾等待时间设定为5秒
					endingDelay = (5) * 1000 / mission.CommonPara.MsPerCycle;
				} else if (state == 1) {
					// 装载10的第一个状态
					for (int i = 0; i < 9; i++) {
						targetVector[i].X = targetVectorX1[i];
						targetVector[i].Z = targetVectorZ1[i];
						targetAngle[i] = deg2rad(targetAngle1[i]);
						targetVector[i].Y = 0;
					}
					//float distance = float.MaxValue;

					// 局部最优的重编号算法
					int p_d = -1; //目标点的编号
					int p_n = -1; //鱼的编号
					float s_d = float.MaxValue; //最小的距离
					for (int i = 0; i < 9; i++) {
						for (int j = 0; j < 9; j++) {
							distance[i, j] = getVectorDistance(targetVector[j], fish[i].PositionMm);
						}
					}

					int count = 9;
					while (count > 0) {

						for (int i = 0; i < 9; i++) {
							//如果这条鱼已经有目标点
							if (FishUse[i])
								continue;
							for (int j = 0; j < 9; j++) {
								//如果这个目标点已经有鱼
								if (DesUse[j])
									continue;
								if (distance[i, j] < s_d) {
									s_d = distance[i, j];
									p_n = i;
									p_d = j;
								}
							}
						}
						DesUse[p_d] = true;
						FishUse[p_n] = true;
						FishDes[p_n] = p_d;
						count--;
						s_d = float.MaxValue;
					}
					//暂时弃用的算法v0.1
					/*for (int i = 0; i < 9; i++)
                    {
                        for (int j = 0; j < 9; j++)
                        {
                            if (DesUse[j])
                                continue;
                            float tmp = getVectorDistance(targetVector[j], fish[i].PositionMm);
                            if (tmp < distance)
                            {
                                distance = tmp;
                                p_d = j;
                            }
                        }
                        FishDes[i] = p_d;
                        DesUse[p_d] = true;
                    }*/

					state = 2;
				} else if (state == 2) {
					// 行动至10的第一个状态

					for (int i = 0; i < 9; i++) {
						fish2Point_Fast(ref decisions[i + 1], fish[i], targetVector[FishDes[i]], targetAngle[FishDes[i]], i + 2, ref timeForPoseToPose, eqFlag);
					}
					if (allEqual(eqFlag, 2, 3, 10)) {
						for (int i = 0; i < 11; i++) {
							eqFlag[i] = 0;
							timeForPoseToPose[i] = 0;
						}
						timer = 0;
						state = 3;
					}

					// 总动作时间超时确认，全部停止运动，跳转到下个项目
					if (timer >= timingLimit) {
						for (int i = 0; i < 9; i++)
							stopFish(ref decisions[i], i + 2);

						stage++;
					}
				} else if (state == 3) {
					if (timer >= endingDelay) {
						// 5秒计时结束
						stage++;
					}
				}

				timer++;
				return;
			}

		}
		#endregion

		#region Chinese_Ren
		//汉字造型“人”
		//陈姝睿
		class Chinese_REN {
			private static int state = 0;

			// 计时器与超时
			private static long timer = 0;
			private static long timingLimit;
			private static int endingDelay;

			static xna.Vector3[] targetVector = new xna.Vector3[9];
			static float[] targetAngle = new float[9];
			// 稳定标记
			private static int[] eqFlag = new int[11];

			private static bool[] DesUse = { false, false, false, false, false, false, false, false, false };
			private static bool[] FishUse = { false, false, false, false, false, false, false, false, false };
			private static int[] FishDes = new int[9];  //鱼的目标点


			/*        X     X     258, -120, -480, -834, -1230, 384, 810, 1242, 1716
        Z     -985, -516, -60, 432, 930, -264, 114, 498, 864
        angle 130, 130, 130, 130, 130, 40, 40, 40, 40 */

			static int[] targetVectorX1 = { 258, -120, -480, -834, -1230, 384, 810, 1242, 1716 };
			static int[] targetVectorZ1 = { -985, -516, -60, 432, 930, -264, 114, 498, 864 };
			static float[] targetAngle1 = { 130, 130, 130, 130, 130, 40, 40, 40, 40 };

			private static float deg2rad(float deg) {
				if (deg > 180) deg -= 360;
				return (float)(Math.PI * deg / 180);
			}

			public static void movingRen(ref Mission mission, int teamId, ref Decision[] decisions) {
				// 仿真周期毫秒数
				int msPerCycle = mission.CommonPara.MsPerCycle;
				RoboFish[] fish = {
					mission.TeamsRef[teamId].Fishes[1],
					mission.TeamsRef[teamId].Fishes[2],
					mission.TeamsRef[teamId].Fishes[3],
					mission.TeamsRef[teamId].Fishes[4],
					mission.TeamsRef[teamId].Fishes[5],
					mission.TeamsRef[teamId].Fishes[6],
					mission.TeamsRef[teamId].Fishes[7],
					mission.TeamsRef[teamId].Fishes[8],
					mission.TeamsRef[teamId].Fishes[9] };

				float[,] distance = new float[9, 9];
				// Reserved，需要重编号功能以尽可能减少时间
				if (state == 0) {
					// 初始化
					state = 1;
					for (int i = 0; i < 11; i++) {
						eqFlag[i] = 0;
						timeForPoseToPose[i] = 0;
					}
					// 时限1分钟
					timingLimit = (2) * 30 * 1000 / mission.CommonPara.MsPerCycle;
					// 结尾等待时间设定为5秒
					endingDelay = (5) * 1000 / mission.CommonPara.MsPerCycle;

				} else if (state == 1) {

					for (int i = 0; i < 9; i++) {
						targetVector[i].X = targetVectorX1[i];
						targetVector[i].Z = targetVectorZ1[i];
						targetAngle[i] = deg2rad(targetAngle1[i]);
						// targetVector[i].Y = 0;
					}

					// 局部最优的重编号算法
					int p_d = -1; //目标点的编号
					int p_n = -1; //鱼的编号
					float s_d = float.MaxValue; //最小的距离
					for (int i = 0; i < 9; i++)
						for (int j = 0; j < 9; j++)
							distance[i, j] = getVectorDistance(targetVector[j], fish[i].PositionMm);

					int count = 9;
					while (count > 0) {
						for (int i = 0; i < 9; i++) {
							//如果这条鱼已经有目标点
							if (FishUse[i])
								continue;
							for (int j = 0; j < 9; j++) {
								//如果这个目标点已经有鱼
								if (DesUse[j])
									continue;
								if (distance[i, j] < s_d) {
									s_d = distance[i, j];
									p_n = i;
									p_d = j;
								}
							}
						}
						DesUse[p_d] = true;
						FishUse[p_n] = true;
						FishDes[p_n] = p_d;
						count--;
						s_d = float.MaxValue;
					}

					state = 2;
				} else if (state == 2) {
					// 行动至10的第一个状态
					for (int i = 0; i < 9; i++) {
							//StrategyHelper.Helpers.PoseToPose(ref decisions[i + 1], fish[i], targetVector[i], targetAngle[i], 30.0f, 10, msPerCycle, ref times); 
							// fish2Point_Fast(ref decisions[i + 1], fish[i], targetVector[i], targetAngle[i], i + 2, ref timeForPoseToPose, eqFlag);
							fish2Point_Fast(ref decisions[i + 1], fish[i], targetVector[FishDes[i]], targetAngle[FishDes[i]], i + 2, ref timeForPoseToPose, eqFlag);
						}
					if (allEqual(eqFlag, 2, 3, 10)) {
						for (int i = 0; i < 11; i++) {
							eqFlag[i] = 0;
							timeForPoseToPose[i] = 0;
						}
						timer = 0;
						state = 3;
					}

					// 总动作时间超时确认，全部停止运动，跳转到下个项目
					if (timer >= timingLimit) {
						for (int i = 0; i < 9; i++)
							stopFish(ref decisions[i], i + 2);

						stage++;
					}
				} else if (state == 3) {
					if (timer >= endingDelay) {
						// 5秒计时结束
						stage++;
					}
				}

				timer++;
				return;
			}

		}
		#endregion

		#region MovingHeart
		/// <summary>
		/// “运动的心”造型类
		/// </summary>
		// 心造型，顺时针旋转，假设开始时所有的鱼接近初始目标点（需要重编号）
		// 模块负责人：联航
		// 07.19: 初始版本，确定心脏的目标点
		// 07.20: 完成动作部分
		// 07.23: 增加5秒延时功能和总动作超时failsafe功能
		// TODO:
		// - 分动作间的延迟太明显了，需要用一些方法让运动更连续，以节约时间
		// - 加入每一分动作的超时failsafe
		class MovingHeartClass {

			// 旋转次数控制
			private static int cycleDownCounter = 5;
			// 状态机
			private static int state = 0;
			// 阶段状态标记
			private static bool heartStageFlag = true;
			// 计时器与超时
			private static long timer = 0;
			private static long timingLimit;
			private static int endingDelay;

			// 稳定标记
			private static int[] eqFlag = new int[11];

			private static xna.Vector3[] targetVector = new xna.Vector3[9];
			private static float[] targetAngle = new float[9];

			// 目标点与角度数据#1
			private static int[] targetVectorX1 = { -1100, -528, 120, 738, 1200, 918, 300, -450, -1100 };
			private static int[] targetVectorZ1 = { -510, -800, -320, -800, -288, 320, 1000, 800, 100 };
			private static float[] targetAngle1 = { 300, 0, 0, 0, 60, 130, 130, 230, 230 };

			// 目标点与角度数据#2
			private static int[] targetVectorX2 = { -1026, -798, -168, 372, 1002, 1032, 642, -60, -648 };
			private static int[] targetVectorZ2 = { -96, -708, -456, -630, -558, 108, 762, 1068, 594 };
			private static float[] targetAngle2 = { 260, 320, 60, 300, 40, 100, 130, 170, 230 };

			private static RoboFish[] fish = new RoboFish[9];

			private static int[] FishDes = new int[9];  //鱼的目标点

			private static bool firstRun = true;

			// 相对编号偏移量，转圈用
			private static int fishIDShift = 0;

			/// <summary>
			/// 获取当前仿真使命（比赛项目）当前队伍所有仿真机器鱼的决策数据构成的数组
			/// </summary>
			/// <param name="mission">服务端当前运行着的仿真使命Mission对象</param>
			/// <param name="teamId">当前队伍在服务端运行着的仿真使命中所处的编号 
			/// 用于作为索引访问Mission对象的TeamsRef队伍列表中代表当前队伍的元素</param>
			/// <returns>当前队伍所有仿真机器鱼的决策数据构成的Decision数组对象</returns>
			public static void movingHeart(ref Mission mission, int teamId, ref Decision[] decisions) {
				// StreamWriter log = new StreamWriter("C:\\log.txt", true);
				// log.Close();

				// 仿真周期毫秒数
				int msPerCycle = mission.CommonPara.MsPerCycle;
				float[,] distance = new float[9, 9];

				for (int i = 0; i < 9; i++)
					fish[i] = mission.TeamsRef[teamId].Fishes[i + 1];

				if (state == 0) {
					// 初始化
					// 计时上限设置为1.5分钟
					// 07.23测试设置为10分钟
					timingLimit = (20) * 30 * 1000 / mission.CommonPara.MsPerCycle;
					// 结尾等待时间设定为5秒
					endingDelay = (5) * 1000 / mission.CommonPara.MsPerCycle;

					for (int i = 0; i < 11; i++) {
						eqFlag[i] = 0;
						timeForPoseToPose[i] = 0;
					}
					state = 1;

				} else if (state == 1) {
					// 装载心的第一个状态
					if (heartStageFlag) {
						for (int i = 0; i < 9; i++) {
							targetVector[i].X = targetVectorX1[i];
							targetVector[i].Z = targetVectorZ1[i];
							targetAngle[i] = deg2rad(targetAngle1[i]);
							targetVector[i].Y = 0;
						}
					} else {
						for (int i = 0; i < 9; i++) {
							targetVector[i].X = targetVectorX2[i];
							targetVector[i].Z = targetVectorZ2[i];
							targetAngle[i] = deg2rad(targetAngle2[i]);
							targetVector[i].Y = 0;
						}
					}

					// 仅在第一次寻找局部最优
					if (firstRun) {
						// 局部最优的重编号算法
						bool[] DesUse = { false, false, false, false, false, false, false, false, false };
						bool[] FishUse = { false, false, false, false, false, false, false, false, false };
						int p_d = -1; //目标点的编号
						int p_n = -1; //鱼的编号
						float s_d = float.MaxValue; //最小的距离
						for (int i = 0; i < 9; i++)
							for (int j = 0; j < 9; j++)
								distance[i, j] = getVectorDistance(targetVector[j], fish[i].PositionMm);
				
						int count = 9;
						while (count > 0) {
							for (int i = 0; i < 9; i++) {
								//如果这条鱼已经有目标点
								if (FishUse[i])
									continue;
								for (int j = 0; j < 9; j++) {
									//如果这个目标点已经有鱼
									if (DesUse[j])
										continue;
									if (distance[i, j] < s_d) {
										s_d = distance[i, j];
										p_n = i;
										p_d = j;
									}
								}
							}
							DesUse[p_d] = true;
							FishUse[p_n] = true;
							FishDes[p_n] = p_d;
							count--;
							s_d = float.MaxValue;
						}
						firstRun = false;
					}
					
					/*
					for (int i = 0; i < 9; i++)
						FishDes[i] = i;
					*/

					state = 2;

				} else if (state == 2) {
					// 行动至心的第一个状态
					for (int i = 0; i < 9; i++) {
						int j;

						if (FishDes[i] + fishIDShift >= 9)
							j = (FishDes[i] + fishIDShift) - 9;
						else
							j = FishDes[i] + fishIDShift;

						if ((fishIDShift == 0) && heartStageFlag)
							// 初始归位
							fish2Point_Fast(ref decisions[i + 1], fish[i], targetVector[j], targetAngle[j], i + 2, ref timeForPoseToPose, eqFlag);
						else
							fish2Point_Forward(ref decisions[i + 1], fish[i], targetVector[j], targetAngle[j], i + 2, ref timeForPoseToPose, eqFlag, 300, deg2rad(20), deg2rad(20), 200);
					}

					if (allEqual(eqFlag, 2, 3, 10)) {
						for (int i = 0; i < 11; i++) {
							eqFlag[i] = 0;
							timeForPoseToPose[i] = 0;
						}
						if (cycleDownCounter == 0) {
							// 旋转结束，为5秒计时清空计时器
							timer = 0;
							state = 4;
						} else {
							// 再转一圈
							cycleDownCounter --;
							state = 3;
						}
					}

					// 总动作时间超时确认，全部停止运动，跳转到下个项目
					if (timer >= timingLimit) {
						for (int i = 0; i < 9; i++)
							stopFish(ref decisions[i], i + 2);

						stage++;
					}

				} else if (state == 3) {
					if (heartStageFlag) fishIDShift++;
					heartStageFlag = !heartStageFlag;
					state = 1;

				} else if (state == 4) { 
					if (timer >= endingDelay) {
						// 5秒计时结束
						stage++;
					}
				}
				timer++;
				return;
			}

		}
		#endregion

		#region interactWithYellowFish
		/// <summary>
		/// “与黄鱼互动”动作类
		/// </summary>
		// 跟在黄鱼后方做“DNA”形运动
		// 模块负责人：？
		// 07.25: 初始版本，只做了一条鱼的，太慢了
		// TODO:
		//  
		class InteractWithYellowFishClass {

			// 三个阶段：
			// 1. 准备，9,10号开始跟随，其他鱼运动到屏幕底端
			// 2. 9,10号进行跟随，持续30s~1min
			// 3. 9,10号向下运动，其他鱼向上运动
			// 4. 9,10号向上运动，其他鱼散开

			// 状态机
			private static int state = 0;
			// 计时器与超时
			private static long timer = 0;
			// 该动作不需要总时限，直接坚持到整个表演的5分钟结束
			private static long[] timingLimit = new long[4];
			private static int endingDelay;

			// 稳定标记
			private static int[] eqFlag = new int[11];

			private static xna.Vector3[] targetVector = new xna.Vector3[9];
			private static float[] targetAngle = new float[9];

			// 一侧围观鱼坐标与位置，另一半对应
			private static int[] staticFishX = { -1730, -1060, -500, 0, 500, 1060, 1730 };
			private static int[] staticFishZ = { 1430, 1320, 1245, 1165, 1245, 1320, 1430 };
			private static float[] staticFishAngle = { 0, -20, -40, -90, -140, -160, -179 };

			private static int[] staticFishX2 = { -450, -300, -150, 0, 150, 300, 450 };
			private static int[] staticFishZ2 = { 300, 200, 100, 0, 100, 200, 300 };

			private static int[] spreadVCode = { 4, 5, 6, 6, 6, 5, 4 };
			private static int[] spreadVCode1 = { 0, 5, 6, 8, 6, 5, 0 };
			private static int[] spreadVCode2 = { 0, 0, 6, 8, 6, 0, 0 };
			private static int[] spreadTCode = { 0, 2, 4, 5, 9, 12, 14 };
			private static int[] spreadTCode1 = { 7, 0, 2, 4, 12, 14, 7 };
			private static int[] spreadTCode2 = { 7, 7, 2, 2, 12, 7, 7 };

			private static bool[] spreadDoneMarker = { false, false, false, false, false, false, false };

			// 运动的鱼与黄鱼的距离，转圈运动速度（增量）
			private const int motionFishProximity = 600;
			private const int mFishInProximity = 150/2;
			private const int motionFishLatency = 400;
			// private const float motionFishAngularDegSpeed = 10;

			// 运动的鱼变量
			private static float motionFishRelativeAngle = 0;
			private static int[] motionFishTargetX = new int[3];
			private static int[] motionFishTargetZ = new int[3];

			private static RoboFish[] fish = new RoboFish[9];

			private static float motionFishArcAngle = 0;
			private static float motionFishArcAngleIncrement;

			/*
			// 黄鱼位置移位寄存器，其中Y保存的是鱼头指向（弧度制）
			// 假设黄鱼运动自身长度的距离总共需要7s
			xna.Vector3[] ramdomFishRegister = new xna.Vector3[10];

			private static void shiftRegPush(ref xna.Vector3[] shiftReg, xna.Vector3 newItem) {
				// shiftReg[0] 是最新的数据
				for (int i = shiftReg.GetLength(0)-1; i > 0; i--)
					shiftReg[i] = shiftReg[i - 1];
				shiftReg[0] = newItem;
				return;
			}
			*/

			public static void interactWithRandomFish(ref Mission mission, int teamId, ref Decision[] decisions) {
				// 仿真周期毫秒数
				int msPerCycle = mission.CommonPara.MsPerCycle;
				// 设定绕黄鱼的鱼运动的速度
				motionFishArcAngleIncrement = (float)Math.PI * (7 * 1000) / (msPerCycle / 2); // 每7s转180°，因为两个周期才执行一次运动，对应msPerCycle要除2

				for (int i = 0; i < 9; i++)
					fish[i] = mission.TeamsRef[teamId].Fishes[i + 1];

				switch (state) {
					case 0: {
							// 初始化
							// 第一阶段计时上限1min
							// 7.26测试设置为0.5min
							timingLimit[0] = (1) * 30 * 1000 / mission.CommonPara.MsPerCycle;
							// 第二阶段计时上限20s?
							timingLimit[1] = (20) * 1000 / mission.CommonPara.MsPerCycle;
							// 结尾等待时间设定为5秒
							endingDelay = (5) * 1000 / mission.CommonPara.MsPerCycle;

							#region // 产生围观鱼的目标点
							for (int i = 0; i < 7; i++) {
								targetVector[i].X = staticFishX[i];
								targetVector[i].Z = staticFishZ[i];
								targetAngle[i] = deg2rad(staticFishAngle[i]);
							} 
							#endregion

							for (int i = 0; i < 11; i++) {
								eqFlag[i] = 0;
								timeForPoseToPose[i] = 0;
							}
							state = 1;
						}
						break;
					case 1: {
							// 第一阶段第一步：产生运动鱼的新坐标
							// Motion #1
							/*
							xna.Vector3 randomFishBasePoint = mission.TeamsRef[teamId].Fishes[0].PositionMm;
							float randomFishBaseDirection = mission.TeamsRef[teamId].Fishes[0].BodyDirectionRad;
							xna.Vector3 randomFishLatencyPoint1 = new xna.Vector3();
							randomFishLatencyPoint1.X = (float)(randomFishBasePoint.X - motionFishLatency * Math.Cos(randomFishBaseDirection));
							randomFishLatencyPoint1.Z = (float)(randomFishBasePoint.Z - motionFishLatency * Math.Sin(randomFishBaseDirection));

							xna.Vector3 randomFishLatencyPoint2 = new xna.Vector3();
							randomFishLatencyPoint2.X = (float)(randomFishBasePoint.X - 2 * motionFishLatency * Math.Cos(randomFishBaseDirection));
							randomFishLatencyPoint2.Z = (float)(randomFishBasePoint.Z - 2 * motionFishLatency * Math.Sin(randomFishBaseDirection));

							xna.Vector3 motionFishVarPoint7 = new xna.Vector3();
							xna.Vector3 motionFishVarPoint8 = new xna.Vector3();
							float motionFishDistance7 = (float)(motionFishProximity * Math.Sin(motionFishArcAngle));
							motionFishVarPoint7.X = (float)(motionFishDistance7 * Math.Sin(randomFishBaseDirection + deg2rad(90)));
							motionFishVarPoint7.Z = (float)(motionFishDistance7 * Math.Cos(randomFishBaseDirection + deg2rad(90)));

							float motionFishDistance8 = (float)(motionFishProximity * Math.Sin(motionFishArcAngle));
							motionFishVarPoint8.X = (float)(motionFishDistance7 * Math.Sin(randomFishBaseDirection + deg2rad(90)));
							motionFishVarPoint8.Z = (float)(motionFishDistance7 * Math.Cos(randomFishBaseDirection + deg2rad(90)));

							targetVector[8] = randomFishLatencyPoint1 + motionFishVarPoint8;
							targetAngle[8] = randomFishBaseDirection;
							targetVector[7] = randomFishLatencyPoint2 + motionFishVarPoint7;
							targetAngle[7] = randomFishBaseDirection;
							*/
							xna.Vector3 rFishBase = mission.TeamsRef[teamId].Fishes[0].PositionMm;
							float rFishBaseDir = mission.TeamsRef[teamId].Fishes[0].BodyDirectionRad;
							xna.Vector3 rFishLPoint = new xna.Vector3();
							rFishLPoint.X = (float)(rFishBase.X - motionFishLatency * Math.Cos(rFishBaseDir));
							rFishLPoint.Z = (float)(rFishBase.Z - motionFishLatency * Math.Sin(rFishBaseDir));

							// xna.Vector3 mFishInProximityVector = new xna.Vector3();
							// mFishInProximityVector.X = (float)(mFishInProximity * Math.Cos(rFishBaseDir + deg2rad(90)));
							// mFishInProximityVector.Z = (float)(mFishInProximity * Math.Sin(rFishBaseDir + deg2rad(90)));

							xna.Vector3 mFishVarPoint7 = new xna.Vector3();
							xna.Vector3 mFishVarPoint8 = new xna.Vector3();
							float mFishDistance7 = ((float)(motionFishProximity * Math.Sin(motionFishArcAngle)));
							mFishDistance7 = (mFishDistance7 > 0f) ? mFishDistance7 : 0;
							mFishVarPoint7.X = (float)((mFishDistance7 + mFishInProximity) * Math.Sin(rFishBaseDir + deg2rad(90)));
							mFishVarPoint7.Z = (float)((mFishDistance7 + mFishInProximity) * Math.Cos(rFishBaseDir + deg2rad(90)));

							float motionFishDistance8 = (float)(motionFishProximity * Math.Sin(motionFishArcAngle));
							mFishVarPoint8.X = -(float)((mFishDistance7 + mFishInProximity) * Math.Sin(rFishBaseDir + deg2rad(90)));
							mFishVarPoint8.Z = -(float)((mFishDistance7 + mFishInProximity) * Math.Cos(rFishBaseDir + deg2rad(90)));

							targetVector[8] = rFishLPoint + mFishVarPoint8;
							targetAngle[8] = rFishBaseDir;
							targetVector[7] = rFishLPoint + mFishVarPoint7;
							targetAngle[7] = rFishBaseDir;

							motionFishArcAngle += motionFishArcAngleIncrement;

							state = 2;
						}
						break;
					case 2: {
							// 第一阶段第二步：开启追鱼模式
							for (int i = 0; i < 7; i++) {
								int j = i;
								fish2Point_Fast(ref decisions[j + 1], fish[j], targetVector[i], targetAngle[i], j + 2, ref timeForPoseToPose, eqFlag);
							}
							fish2Point_Fast(ref decisions[8], fish[7], targetVector[7], targetAngle[7], 9, ref timeForPoseToPose, eqFlag);
							fish2Point_Fast(ref decisions[9], fish[8], targetVector[8], targetAngle[8], 10, ref timeForPoseToPose, eqFlag);
							
							// 第一阶段动作时间超时确认，停止运动，进入下个阶段
							if (timer >= timingLimit[0]) {
								for (int i = 0; i < 9; i++)
									stopFish(ref decisions[i], i + 2);
								state = 3;
							} else
								state = 1;

						}
						break;
					case 3: {
							// 第二阶段第一步，产生目标点
							// 围观鱼运动到场地中间
							for (int i = 0; i < 7; i++) {
								targetVector[i].X = staticFishX2[i];
								targetVector[i].Z = staticFishZ2[i];
								targetAngle[i] = deg2rad(-90);
							}
							// 前一阶段陪黄鱼玩的两条鱼运动到底下
							targetVector[7].X = -250;
							targetVector[7].Z = 1300;
							targetAngle[7] = deg2rad(60);
							targetVector[8].X = 250;
							targetVector[8].Z = 1300;
							targetAngle[8] = deg2rad(120);

							for (int i = 0; i < 11; i++) {
								eqFlag[i] = 0;
								timeForPoseToPose[i] = 0;
							}
							timer = 0;
							state = 4;
						}
						break;
					case 4: {
							// 第二阶段第二步：运动，除了9，10外到了就进入下个阶段
							for (int i = 0; i < 8; i++) {
								int j = i;
								fish2Point_Fast(ref decisions[j + 1], fish[j], targetVector[i], targetAngle[i], j + 2, ref timeForPoseToPose, eqFlag);
							}
							fish2Point_Fast(ref decisions[9], fish[8], targetVector[8], targetAngle[8], 10, ref timeForPoseToPose, eqFlag);
							if (allEqual(eqFlag, 2, 3, 8)) {
								for (int i = 0; i < 9; i++) {
									eqFlag[i] = 0;
									timeForPoseToPose[i] = 0;
								}
								state = 5;
							}
						}
						break;
					case 5: {
							// 散开
							for (int i = 1; i < mission.CommonPara.FishCntPerTeam-2; i++) {
								if (isDirectionInRange(deg2rad(90), mission.TeamsRef[teamId].Fishes[i].VelocityDirectionRad, 0.4f)==0)
									spreadDoneMarker[i - 1] = true;

								if (spreadDoneMarker[i - 1]) {
									targetVector[i - 1].X = mission.TeamsRef[teamId].Fishes[i].PositionMm.X;
									targetVector[i - 1].Z = 1300;
									targetAngle[i - 1] = deg2rad(90);
									fish2Point_Fast(ref decisions[i], fish[i - 1], targetVector[i - 1], targetAngle[i - 1], i + 1, ref timeForPoseToPose, eqFlag);
								} else {
									if (spreadDoneMarker[0] && spreadDoneMarker[6]) {
										if (spreadDoneMarker[1] && spreadDoneMarker[5]) {
											decisions[i].VCode = spreadVCode2[i - 1];
											decisions[i].TCode = spreadTCode2[i - 1];
										} else {
											decisions[i].VCode = spreadVCode1[i - 1];
											decisions[i].TCode = spreadTCode1[i - 1];
										}
									} else {
										decisions[i].VCode = spreadVCode[i - 1];
										decisions[i].TCode = spreadTCode[i - 1];
									}
								}
							}
						}
						break;
					default: {
							for (int i = 0; i < 9; i++)
								stopFish(ref decisions[i], i + 2);
						}
						break;
				}
				timer++;

			}


		}
		#endregion

		#region fireworks
		/// <summary>
		///  "烟花"动作类
		/// </summary>
		class FireworksClass {
			// 状态机
			private static int state = 0;
			// 计时器与超时
			private static long timer = 0;
			private static long timingLimit;
			private static int endingDelay;

			// 稳定标记
			private static int[] eqFlag = new int[11];

			private static xna.Vector3[] targetVector = new xna.Vector3[9];
			private static float[] targetAngle = new float[9];
		}
		#endregion


		public Decision[] GetDecision(Mission mission, int teamId) {
			// 决策类当前对象第一次调用GetDecision时Decision数组引用为null
			if (decisions == null) {// 根据决策类当前对象对应的仿真使命参与队伍仿真机器鱼的数量分配决策数组空间
				decisions = new Decision[mission.CommonPara.FishCntPerTeam];
				preDecisions = new Decision[mission.CommonPara.FishCntPerTeam];
			}
			mission.CommonPara.MsPerCycle = 100;
			#region 决策计算过程 需要各参赛队伍实现的部分
			#region 策略编写帮助信息
			//====================我是华丽的分割线====================//
			//======================策略编写指南======================//
			//1.策略编写工作直接目标是给当前队伍决策数组decisions各元素填充决策值
			//2.决策数据类型包括两个int成员，VCode为速度档位值，TCode为转弯档位值
			//3.VCode取值范围0-14共15个整数值，每个整数对应一个速度值，速度值整体但非严格递增
			//有个别档位值对应的速度值低于比它小的档位值对应的速度值，速度值数据来源于实验
			//4.TCode取值范围0-14共15个整数值，每个整数对应一个角速度值
			//整数7对应直游，角速度值为0，整数6-0，8-14分别对应左转和右转，偏离7越远，角度速度值越大
			//5.任意两个速度/转弯档位之间切换，都需要若干个仿真周期，才能达到稳态速度/角速度值
			//目前运动学计算过程决定稳态速度/角速度值接近但小于目标档位对应的速度/角速度值
			//6.决策类Strategy的实例在加载完毕后一直存在于内存中，可以自定义私有成员变量保存必要信息
			//但需要注意的是，保存的信息在中途更换策略时将会丢失
			//====================我是华丽的分割线====================//
			//=======策略中可以使用的比赛环境信息和过程信息说明=======//
			//场地坐标系: 以毫米为单位，矩形场地中心为原点，向右为正X，向下为正Z
			//            负X轴顺时针转回负X轴角度范围为(-PI,PI)的坐标系，也称为世界坐标系
			//mission.CommonPara: 当前仿真使命公共参数
			//mission.CommonPara.FishCntPerTeam: 每支队伍仿真机器鱼数量
			//mission.CommonPara.MsPerCycle: 仿真周期毫秒数
			//mission.CommonPara.RemainingCycles: 当前剩余仿真周期数
			//mission.CommonPara.TeamCount: 当前仿真使命参与队伍数量
			//mission.CommonPara.TotalSeconds: 当前仿真使命运行时间秒数
			//mission.EnvRef.Balls: 
			//当前仿真使命涉及到的仿真水球列表，列表元素的成员意义参见URWPGSim2D.Common.Ball类定义中的注释
			//mission.EnvRef.FieldInfo: 
			//当前仿真使命涉及到的仿真场地，各成员意义参见URWPGSim2D.Common.Field类定义中的注释
			//mission.EnvRef.ObstaclesRect: 
			//当前仿真使命涉及到的方形障碍物列表，列表元素的成员意义参见URWPGSim2D.Common.RectangularObstacle类定义中的注释
			//mission.EnvRef.ObstaclesRound:
			//当前仿真使命涉及到的圆形障碍物列表，列表元素的成员意义参见URWPGSim2D.Common.RoundedObstacle类定义中的注释
			//mission.TeamsRef[teamId]:
			//决策类当前对象对应的仿真使命参与队伍（当前队伍）
			//mission.TeamsRef[teamId].Para:
			//当前队伍公共参数，各成员意义参见URWPGSim2D.Common.TeamCommonPara类定义中的注释
			//mission.TeamsRef[teamId].Fishes:
			//当前队伍仿真机器鱼列表，列表元素的成员意义参见URWPGSim2D.Common.RoboFish类定义中的注释
			//mission.TeamsRef[teamId].Fishes[i].PositionMm和PolygonVertices[0],BodyDirectionRad,VelocityMmPs,
			//                                   AngularVelocityRadPs,Tactic:
			//当前队伍第i条仿真机器鱼鱼体矩形中心和鱼头顶点在场地坐标系中的位置（用到X坐标和Z坐标），鱼体方向，速度值，
			//                                   角速度值，决策值
			//====================我是华丽的分割线====================//
			//========================典型循环========================//
			//for (int i = 0; i < mission.CommonPara.FishCntPerTeam; i++)
			//{
			//  decisions[i].VCode = 0; // 静止
			//  decisions[i].TCode = 7; // 直游
			//}
			//====================我是华丽的分割线====================//
			#endregion

			// 表演顺序：
			//  Ⅰ.标准动作阶段
			// #0   1. 一个包含阿拉伯数字的造型：数字10
			// #1   2. 一个包含汉字的造型：人
			// #2   3. 动态封闭图形： 运动的心
			//  II.自由动作阶段
			// #3   1.黄鱼互动：？

			switch (stage) {
				case 0:
					NumberTenClass.movingTen(ref mission, teamId, ref decisions);
					break;
				case 1:
					Chinese_REN.movingRen(ref mission, teamId, ref decisions);
					break;
				case 2:
					MovingHeartClass.movingHeart(ref mission, teamId, ref decisions);
					break;
				case 3: // SurroundRandomFishClass.surroundRandomFish(ref mission, teamId, ref decisions);
					InteractWithYellowFishClass.interactWithRandomFish(ref mission, teamId, ref decisions);
					break;
				case 4:
				default:
					completeFlag = true;
					break;
			}

			#endregion
			return decisions;
		}
	}
}