using System;
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

            if (a - b > 0.15)
                return 1;
            else if (a - b < -0.15)
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
			if (deg > 180) deg -= 360;
			return (float)(Math.PI * deg / 180);
		}

		public static void fishToPoint(ref Decision decisions, RoboFish fish, xna.Vector3 targetePoint, float targetDirection, int noOfFish, ref int[] timeForPoseToPose, int[] flag) {
            switch (flag[noOfFish]) {
                case 0:
                    if (getVectorDistance(targetePoint, fish.PositionMm) > 100) {
                        Helpers.PoseToPose(ref decisions, fish, targetePoint, targetDirection, 6f, 10f, 100, ref timeForPoseToPose[noOfFish]);
                    }
                    //  Helpers.PoseToPose(ref decisions, fish, targetePoint, targetDirection, 6f, 10f, 50, ref timeForPoseToPose[noOfFish]);
                    if (getVectorDistance(targetePoint, fish.PositionMm) < 100) {
                        flag[noOfFish] = 1;
                    }
                    break;
                case 1:
                    if (isDirectionRight(targetDirection, fish.BodyDirectionRad) < 0) {
                        decisions.TCode = 0;
                        decisions.VCode = 1;
                    } else if (isDirectionRight(targetDirection, fish.BodyDirectionRad) > 0) {
                        decisions.TCode = 14;
                        decisions.VCode = 1;
                    } else {
                        flag[noOfFish] = 2;
                        stopFish(ref decisions, noOfFish);
                    }
                    if (getVectorDistance(targetePoint, fish.PositionMm) > 150)
                        flag[noOfFish] = 0;
                    break;
                case 2:
                    if (isDirectionRight(targetDirection, fish.BodyDirectionRad) < 0) {
                        decisions.TCode = 0;
                        decisions.VCode = 1;
                    } else if (isDirectionRight(targetDirection, fish.BodyDirectionRad) > 0) {
                        decisions.TCode = 14;
                        decisions.VCode = 1;
                    } else {
                        stopFish(ref decisions, noOfFish);
                    }
                    if (getVectorDistance(targetePoint, fish.PositionMm) > 150)
                        flag[noOfFish] = 0;
                    break;
                default:
                    stopFish(ref decisions, noOfFish);
                    break;
            }
        }

        public static void fishToPointQuick(ref Decision decisions, RoboFish fish, xna.Vector3 targetePoint, float targetDirection, int noOfFish, ref int[] timeForPoseToPose, int[] flag) {
            switch (flag[noOfFish]) {
                case 0:
                    if (getVectorDistance(targetePoint, fish.PositionMm) > 180)
                        Helpers.PoseToPose(ref decisions, fish, targetePoint, targetDirection, 45f, 50f, 100, ref timeForPoseToPose[noOfFish]);
                    if (getVectorDistance(targetePoint, fish.PositionMm) <= 180)
                        flag[noOfFish] = 1;
                    break;
                case 1:
                    if (getVectorDistance(targetePoint, fish.PositionMm) > 250)
                        flag[noOfFish] = 0;
                    else if (isDirectionRight(targetDirection, fish.BodyDirectionRad) < 0) {
                        decisions.TCode = 0;
                        decisions.VCode = 1;
                    } else if (isDirectionRight(targetDirection, fish.BodyDirectionRad) > 0) {
                        decisions.TCode = 14;
                        decisions.VCode = 1;
                    } else {
                        flag[noOfFish] = 2;
                        stopFish(ref decisions, noOfFish);
                    }
                    break;
                case 2:
                    if (getVectorDistance(targetePoint, fish.PositionMm) > 250)
                        flag[noOfFish] = 0;
                    else if (isDirectionRight(targetDirection, fish.BodyDirectionRad) != 0)
                        flag[noOfFish] = 1;
                    else
                        stopFish(ref decisions, noOfFish);
                    break;
                default:
                    stopFish(ref decisions, noOfFish);
                    break;
            }
        }

        public static int completeCircle = 0;
        Decision[] preDecisions = null;

        // 表演阶段标记
        private static int stage = 2;

        private static int timeflag = 0;
        private static int[] timeForPoseToPose = new int[11];
        private static bool completeFlag = false;

        #region NumberTen
        //阿拉伯数字造型10
        //程钰
        //v0.2 07.24:加入最近算法
        //运气不好的话还是会一直转圈
        //我感觉fishToPoint可以修改一下？让他不要那么蠢总是转转转
        //算法还没debug，肉眼debug没啥问题= =
        class NumberTenClass
        {
            private static int state = 0;
            private static int times = 0;
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

            private static float deg2rad(float deg)
            {
                if (deg > 180) deg -= 360;
                return (float)(Math.PI * deg / 180);
            }


            public static void movingTen(ref Mission mission, int teamId, ref Decision[] decisions)
            {
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
                if (state == 0)
                {
                    // 初始化
                    state = 1;
                    for (int i = 0; i < 11; i++)
                    {
                        eqFlag[i] = 0;
                        timeForPoseToPose[i] = 0;
                    }
                    return;
                }
                else if (state == 1)
                {
                    // 装载10的第一个状态
                    for (int i = 0; i < 9; i++)
                    {
                        targetVector[i].X = targetVectorX1[i];
                        targetVector[i].Z = targetVectorZ1[i];
                        targetAngle[i] = deg2rad(targetAngle1[i]);
                        targetVector[i].Y = 0;
                    }
                    //float distance = float.MaxValue;
                    int p_d = -1; //目标点的编号
                    int p_n = -1; //鱼的编号
                    float s_d = float.MaxValue; //最小的距离
                    for (int i = 0; i < 9; i++)
                    {
                        for (int j = 0; j < 9; j++)
                        {
                            distance[i, j] = getVectorDistance(targetVector[j], fish[i].PositionMm);
                        }
                    }
                    int count = 9;
                    while (count > 0)
                    {

                        for (int i = 0; i < 9; i++)
                        {
                            //如果这条鱼已经有目标点
                            if (FishUse[i])
                                continue;
                            for (int j = 0; j < 9; j++)
                            {
                                //如果这个目标点已经有鱼
                                if (DesUse[j])
                                    continue;
                                if (distance[i, j] < s_d)
                                {
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
                    return;
                }
                else if (state == 2)
                {
                    // 行动至10的第一个状态

                    for (int i = 0; i < 9; i++)
                    {
                        //StrategyHelper.Helpers.PoseToPose(ref decisions[i + 1], fish[i], targetVector[i], targetAngle[i], 30.0f, 10, msPerCycle, ref times); 
                        fishToPoint(ref decisions[i + 1], fish[i], targetVector[FishDes[i]], targetAngle[i], i + 2, ref timeForPoseToPose, eqFlag);
                    }
                    /* if (allEqual(eqFlag, 2, 3, 10))
                     {
                         for (int i = 0; i < 11; i++)
                         {
                             eqFlag[i] = 0;
                             timeForPoseToPose[i] = 0;
                         }
                         state = 1;
                     }*/
                    return;
                }
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
			private static int cycleDownCounter = 2;
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
			private static float[] targetAngle2 = { 260, 320, 60, 300, 40, 100, 130, 180, 230 };

			private static RoboFish[] fish = new RoboFish[9];

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

				for (int i = 0; i < 9; i++)
					fish[i] = mission.TeamsRef[teamId].Fishes[i + 1];
				// Reserved，需要重编号功能以尽可能减少时间
				if (state == 0) {
					// 初始化
					// 计时上限设置为1.5分钟
					// 07.23测试设置为0.5分钟
					timingLimit = (1) * 30 * 1000 / mission.CommonPara.MsPerCycle;
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
					state = 2;

				} else if (state == 2) {
					// 行动至心的第一个状态
					for (int i = 0; i < 9; i++) {
						int j;
						if (i - fishIDShift < 0) j = 9 + i - fishIDShift;
						else j = i - fishIDShift;
						fishToPointQuick(ref decisions[j + 1], fish[j], targetVector[i], targetAngle[i], j + 2, ref timeForPoseToPose, eqFlag);
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
							cycleDownCounter -= 1;
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
                case 0: NumberTenClass.movingTen(ref mission, teamId, ref decisions);
                    break;
                case 1: stage++; // 尚未完成
                    break;
                case 2: MovingHeartClass.movingHeart(ref mission, teamId, ref decisions);
                    break;
                case 3: stage++; // 尚未完成
                    break;
                case 4: 
                default: completeFlag = true;
                    break;
            }

            #endregion
            return decisions;
        }
    }
}




