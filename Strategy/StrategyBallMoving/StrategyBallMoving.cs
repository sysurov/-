using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using xna = Microsoft.Xna.Framework;
using URWPGSim2D.Common;
using URWPGSim2D.StrategyLoader;
using URWPGSim2D.StrategyHelper;
using System.IO;


namespace URWPGSim2D.Strategy

{
    public class Strategy : MarshalByRefObject, IStrategy
    {
        #region reserved code never be changed or removed

        /// <summary>

        /// override the InitializeLifetimeService to return null instead of a valid ILease implementation

        /// to ensure this type of remote object never dies

        /// </summary>

        /// <returns>null</returns>

        public override object InitializeLifetimeService()
        {
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

        public string GetTeamName()
        {
            return "中东检修队";
        }

        public int times = 0;
        public float ballR = 58;

        private xna.Vector3 p0; //0号球中转点
        private xna.Vector3 p4; //4号球中转点

        private int CycleTime = 100;  //仿真周期
        public int state1 = 0;  //记录鱼的状态
        public int state2 = 0;
        //StreamWriter log = new StreamWriter("C:\\Users\\Jade\\Desktop\\URWPGSim2D\\Log.txt", true);  

        public bool[] inHole = { false, false, false, false, false, false };

        #region 鱼和球相关变量声明

        private xna.Vector3[] ball = new xna.Vector3[6];    //球的坐标
        private xna.Vector3[] hole = new xna.Vector3[6];  //定义6个地标的中心坐标，记录在hole数组中
        private void CalHol()
        {
            for (int i = 1; i < 4; i++)
                for (int j = 1; j <= i; j++)
                {
                    int l = i * (i - 1) / 2 + j - 1;
                    hole[l].Y = 0;
                    hole[l].X = -(i * 3 - 3) * 80;
                    hole[l].Z = -(i - 1) * 2 * 80 + 4 * (j - 1) * 80;
                }
        }

        private RoboFish rFish1;
        private RoboFish rFish2;

        private static int timeControl1 = 0;
        private static int timeControl2 = 0;

        public struct myFish
        {
            public xna.Vector3 head;   //鱼头坐标
            public xna.Vector3 body;   //鱼体坐标
            public xna.Vector3 Position;   //刚体中心
            public float Direction;    //鱼的方向（Rad）
            public float velocity;     //当前速度
        }

        myFish my_fish1, my_fish2;
        #endregion

        public double getDistance(xna.Vector3 temp1, xna.Vector3 temp2)   //点与点的最短距离
        {
            return Math.Sqrt(Math.Pow(temp1.X - temp2.X, 2d) + Math.Pow(temp1.Z - temp2.Z, 2d));
        }

        public float deg2rad(float deg)
        {
            if (deg > 180) deg -= 360;
            return (float)(Math.PI * deg / 180);
        }

        //鱼2的运动
        public void do_fish2(Mission mission, int teamId)
        {
            CalHol();
            CycleTime = mission.CommonPara.MsPerCycle;

            rFish2 = mission.TeamsRef[teamId].Fishes[1];
            my_fish2.head = mission.TeamsRef[teamId].Fishes[1].PolygonVertices[0];
            my_fish2.body = new xna.Vector3((mission.TeamsRef[teamId].Fishes[1].PolygonVertices[0].X + mission.TeamsRef[teamId].Fishes[1].PolygonVertices[3].X) / 2, 0, (mission.TeamsRef[teamId].Fishes[1].PolygonVertices[0].Z + mission.TeamsRef[teamId].Fishes[1].PolygonVertices[3].Z) / 2);
            my_fish2.Position = mission.TeamsRef[teamId].Fishes[1].PositionMm;
            my_fish2.Direction = mission.TeamsRef[teamId].Fishes[1].BodyDirectionRad;

            ball[0] = mission.EnvRef.Balls[0].PositionMm;
            ball[1] = mission.EnvRef.Balls[1].PositionMm;
            ball[3] = mission.EnvRef.Balls[3].PositionMm;
            ball[2] = mission.EnvRef.Balls[2].PositionMm;
            ball[5] = mission.EnvRef.Balls[5].PositionMm;
            ball[4] = mission.EnvRef.Balls[4].PositionMm;

            int b2 = Convert.ToInt32(mission.HtMissionVariables["Ball2InHole"]);
            int b5 = Convert.ToInt32(mission.HtMissionVariables["Ball5InHole"]);
            int b4 = Convert.ToInt32(mission.HtMissionVariables["Ball4InHole"]);
            int b0 = Convert.ToInt32(mission.HtMissionVariables["Ball0InHole"]);
            int b1 = Convert.ToInt32(mission.HtMissionVariables["Ball1InHole"]);
            int b3 = Convert.ToInt32(mission.HtMissionVariables["Ball3InHole"]);



            switch (state2)
            {
                case 0://来到画面右上方
                    //xna.Vector3 temp = new xna.Vector3(1048, 0 ,-556);
                    xna.Vector3 temp = new xna.Vector3(ball[3].X + (float)2 * ballR, 0, ball[3].Z - (float)3 * ballR);
                    Helpers.PoseToPose(ref decisions[1], rFish2, temp, deg2rad(30), 30, 100, CycleTime, ref times);
                    if (getDistance(temp, my_fish2.body) < 100)
                    {
                        state2++;
                        times = 0;
                    }
                    break;

                case 1: //转向球3的右上方
                    temp = new xna.Vector3(ball[3].X + (float)1.5 * ballR, 0, ball[3].Z - (float)1.5 * ballR);
                    float angle = xna.MathHelper.ToRadians((float)Helpers.GetAngleDegree(hole[3] - ball[3])); //目标点与球的方向
                    Helpers.PoseToPose(ref decisions[1], rFish2, temp, angle, 30, 50, CycleTime, ref times);
                    if (getDistance(temp, my_fish2.body) < 100)
                    {
                        state2++;
                        times = 0;
                    }
                    break;

                case 2://将球3向外推动
                    //temp = new xna.Vector3(ball[3].X - 2 * ballR, 0, ball[3].Z - 3*ballR);
                    angle = xna.MathHelper.ToRadians((float)Helpers.GetAngleDegree(hole[3] - ball[3])); //目标点与球的方向
                    temp = new xna.Vector3(ball[3].X - (float)1.1 * ballR * (float)Math.Cos(angle), 0, ball[3].Z - (float)1.1 * ballR * (float)Math.Sin(angle));
                    Helpers.Dribble(ref decisions[1], rFish2, temp, angle, 6, 6, 150, 7, 5, 5, CycleTime, true);
                    timeControl2++;
                    if (b3 == 1)
                    {
                        state2++;
                        times = 0;
                        inHole[3] = true;
                        timeControl2 = 0;
                        if (inHole[0])
                            state2 = 8;
                    }

                    if (timeControl2 > 1000)
                    {
                        timeControl2 = 0;
                        state2++;
                        times = 0;
                    }
                    break;
                case 3://游到球0上方
                    temp = new xna.Vector3(ball[0].X - (float)1 * ballR, 0, ball[0].Z - (float)3 * ballR);
                    Helpers.PoseToPose(ref decisions[1], rFish2, temp, deg2rad(90), 30, 100, CycleTime, ref times);
                    if (getDistance(temp, my_fish2.body) < 100)
                    {
                        state2++;
                        times = 0;
                        p0 = new xna.Vector3(ball[0].X - ballR, 0, ball[0].Z + 3 * ballR);
                    }
                    break;
                case 4://推球0到球下方的位置
                    angle = xna.MathHelper.ToRadians((float)Helpers.GetAngleDegree(p0 - ball[0])); //目标点与鱼的方向
                    temp = new xna.Vector3(ball[0].X - (float)1.1 * ballR * (float)Math.Cos(angle), 0, ball[0].Z - (float)1.1 * ballR * (float)Math.Sin(angle));
                    Helpers.Dribble(ref decisions[1], rFish2, temp, angle, 6, 6, 150, 7, 5, 5, CycleTime, true);
                    if (getDistance(p0, my_fish2.body) < 100)
                    {
                        state2++;
                        times = 0;
                    }
                    break;
                case 5://将鱼0推入洞
                    angle = xna.MathHelper.ToRadians((float)Helpers.GetAngleDegree(hole[0] - ball[0])); //目标点与鱼的方向
                    temp = new xna.Vector3(ball[0].X - (float)1.1 * ballR * (float)Math.Cos(angle), 0, ball[0].Z - (float)1.1 * ballR * (float)Math.Sin(angle));
                    Helpers.Dribble(ref decisions[1], rFish2, temp, angle, 6, 6, 150, 7, 5, 5, CycleTime, true);
                    timeControl2++;
                    if (b0 == 1)
                    {
                        state2++;
                        times = 0;
                        inHole[0] = true;
                        timeControl2 = 0;
                        if (inHole[1])
                            state2 = 8;
                    }

                    if (timeControl2 > 1000)
                    {
                        timeControl2 = 0;
                        state2++;
                        times = 0;
                    }
                    break;
                case 6: //移动到球1的后方
                    temp = new xna.Vector3(ball[1].X + (float)1.5 * ballR, 0, ball[1].Z + (float)1.5 * ballR);
                    Helpers.PoseToPose(ref decisions[1], rFish2, temp, deg2rad(0), 30, 100, CycleTime, ref times);
                    if (getDistance(temp, my_fish2.body) < 100)
                    {
                        state2++;
                        times = 0;
                    }
                    break;

                case 7:
                    angle = xna.MathHelper.ToRadians((float)Helpers.GetAngleDegree(hole[1] - ball[1])); //目标点与鱼的方向
                    temp = new xna.Vector3(ball[1].X - (float)1.1 * ballR * (float)Math.Cos(angle), 0, ball[1].Z - (float)1.1 * ballR * (float)Math.Sin(angle));
                    Helpers.Dribble(ref decisions[1], rFish2, temp, angle, 6, 6, 150, 7, 5, 5, CycleTime, true);
                    timeControl2++;
                    if (b1 == 1)
                    {
                        state2++;
                        times = 0;
                        inHole[1] = true;
                        timeControl2 = 0;
                    }

                    if (timeControl2 > 1000)
                    {
                        state2++;
                        times = 0;
                        timeControl2 = 0;
                    }
                    break;

                case 8:
                    if (!inHole[3])
                        state2 = 0;
                    if (!inHole[0])
                        state2 = 3;
                    if (!inHole[1])
                        state2 = 6;

                    decisions[1].VCode = 0;
                    decisions[1].TCode = 0;
                    break;
            }
        }

        //鱼1的运动
        public void do_fish1(Mission mission, int teamId)
        {
            CalHol();
            CycleTime = mission.CommonPara.MsPerCycle;

            rFish1 = mission.TeamsRef[teamId].Fishes[0];
            my_fish1.head = rFish1.PolygonVertices[0];
            my_fish1.body = new xna.Vector3((rFish1.PolygonVertices[0].X + rFish1.PolygonVertices[4].X) / 2, 0, (rFish1.PolygonVertices[0].Z + rFish1.PolygonVertices[4].Z) / 2);
            my_fish1.Position = rFish1.PositionMm;
            my_fish1.Direction = rFish1.BodyDirectionRad;

            ball[0] = mission.EnvRef.Balls[0].PositionMm;
            ball[1] = mission.EnvRef.Balls[1].PositionMm;
            ball[3] = mission.EnvRef.Balls[3].PositionMm;
            ball[2] = mission.EnvRef.Balls[2].PositionMm;
            ball[5] = mission.EnvRef.Balls[5].PositionMm;
            ball[4] = mission.EnvRef.Balls[4].PositionMm;

            int b2 = Convert.ToInt32(mission.HtMissionVariables["Ball2InHole"]);
            int b5 = Convert.ToInt32(mission.HtMissionVariables["Ball5InHole"]);
            int b4 = Convert.ToInt32(mission.HtMissionVariables["Ball4InHole"]);
            int b0 = Convert.ToInt32(mission.HtMissionVariables["Ball0InHole"]);
            int b1 = Convert.ToInt32(mission.HtMissionVariables["Ball1InHole"]);
            int b3 = Convert.ToInt32(mission.HtMissionVariables["Ball3InHole"]);


            switch (state1)
            {

                case 0://来到画面右下方
                    xna.Vector3 temp = new xna.Vector3(ball[5].X + (float)2 * ballR, 0, ball[5].Z + (float)3 * ballR);
                    Helpers.PoseToPose(ref decisions[0], rFish1, temp, deg2rad(-30), 30, 100, CycleTime, ref times);
                    if (getDistance(temp, my_fish1.body) < 100)
                    {
                        state1++;
                        times = 0;
                    }
                    break;

                case 1: //转向球5的右下方
                    temp = new xna.Vector3(ball[5].X + (float)1.5 * ballR, 0, ball[5].Z + (float)1.5 * ballR);
                    float angle = xna.MathHelper.ToRadians((float)Helpers.GetAngleDegree(hole[5] - ball[5])); //目标点与球的方向
                    Helpers.PoseToPose(ref decisions[0], rFish1, temp, angle, 30, 50, CycleTime, ref times);
                    if (getDistance(temp, my_fish1.body) < 100)
                    {
                        state1++;
                        times = 0;
                    }
                    break;

                case 2://将球推进洞

                    angle = xna.MathHelper.ToRadians((float)Helpers.GetAngleDegree(hole[5] - ball[5])); //目标点与鱼的方向
                    temp = new xna.Vector3(ball[5].X - (float)1.1 * ballR * (float)Math.Cos(angle), 0, ball[5].Z - (float)1.1 * ballR * (float)Math.Sin(angle));
                    Helpers.Dribble(ref decisions[0], rFish1, temp, angle, 6, 6, 150, 7, 5, 5, CycleTime, true);
                    timeControl1++;
                    if (b5 == 1)
                    {
                        state1++;
                        times = 0;
                        inHole[5] = true;
                        timeControl1 = 0;
                        if (inHole[4])
                            state1 = 8;
                    }

                    if (timeControl1 > 1000)
                    {
                        state1++;
                        times = 0;
                        timeControl1 = 0;
                    }
                    break;

                case 3://游到球4下方
                    temp = new xna.Vector3(ball[4].X + (float)0.5 * ballR, 0, ball[4].Z + (float)3 * ballR);
                    Helpers.PoseToPose(ref decisions[0], rFish1, temp, deg2rad(270), 30, 100, CycleTime, ref times);
                    if (getDistance(temp, my_fish1.body) < 100)
                    {
                        p4 = new xna.Vector3(ball[4].X - ballR, 0, ball[4].Z - 3 * ballR);
                        state1++;
                        times = 0;
                    }
                    break;
                case 4://推鱼到鱼上方的位置
                    angle = xna.MathHelper.ToRadians((float)Helpers.GetAngleDegree(p4 - ball[4])); //目标点与鱼的方向
                    temp = new xna.Vector3(ball[4].X - (float)1.1 * ballR * (float)Math.Cos(angle), 0, ball[4].Z - (float)1.1 * ballR * (float)Math.Sin(angle));
                    Helpers.Dribble(ref decisions[0], rFish1, temp, angle, 6, 6, 150, 7, 5, 5, CycleTime, true);
                    if (getDistance(p4, my_fish1.body) < 100)
                    {
                        state1++;
                        times = 0;
                    }
                    break;
                case 5: //将球4推入洞
                    angle = xna.MathHelper.ToRadians((float)Helpers.GetAngleDegree(hole[4] - ball[4])); //目标点与鱼的方向
                    temp = new xna.Vector3(ball[4].X - (float)1.1 * ballR * (float)Math.Cos(angle), 0, ball[4].Z - (float)1.1 * ballR * (float)Math.Sin(angle));
                    Helpers.Dribble(ref decisions[0], rFish1, temp, angle, 6, 6, 150, 7, 5, 5, CycleTime, true);
                    timeControl1++;
                    if (b4 == 1)
                    {
                        state1++;
                        times = 0;
                        inHole[4] = true;
                        timeControl1 = 0;
                        if (inHole[2])
                            state1 = 8;
                    }
                    if (timeControl1 > 1000)
                    {
                        state1++;
                        times = 0;
                        timeControl1 = 0;
                    }
                    break;
                case 6: //移动到球2的后方
                    temp = new xna.Vector3(ball[2].X + (float)1.5 * ballR, 0, ball[2].Z + (float)1.5 * ballR);
                    Helpers.PoseToPose(ref decisions[0], rFish1, temp, deg2rad(0), 30, 100, CycleTime, ref times);
                    if (getDistance(temp, my_fish1.body) < 100)
                    {
                        state1++;
                        times = 0;
                    }
                    break;
                case 7:
                    angle = xna.MathHelper.ToRadians((float)Helpers.GetAngleDegree(hole[2] - ball[2])); //目标点与鱼的方向
                    temp = new xna.Vector3(ball[2].X - (float)1.1 * ballR * (float)Math.Cos(angle), 0, ball[2].Z - (float)1.1 * ballR * (float)Math.Sin(angle));
                    Helpers.Dribble(ref decisions[0], rFish1, temp, angle, 6, 6, 150, 7, 5, 5, CycleTime, true);
                    timeControl1++;
                    if (b2 == 1)
                    {
                        state1++;
                        times = 0;
                        inHole[2] = true;
                        timeControl1 = 0;
                    }

                    if (timeControl1 > 1000)
                    {
                        state1++;
                        times = 0;
                        timeControl1 = 0;
                    }
                    break;
                case 8:
                    if (!inHole[5])
                        state1 = 0;
                    if (!inHole[4])
                        state1 = 3;
                    if (!inHole[2])
                        state1 = 6;

                    decisions[0].VCode = 0;
                    decisions[0].TCode = 0;
                    break;

            }
        }

        public Decision[] GetDecision(Mission mission, int teamId)

        {
            // 决策类当前对象第一次调用GetDecision时Decision数组引用为null
            if (decisions == null)
            {
                // 根据决策类当前对象对应的仿真使命参与队伍仿真机器鱼的数量分配决策数组空间
                decisions = new Decision[mission.CommonPara.FishCntPerTeam];
            }

            mission.CommonPara.MsPerCycle = 100;
            ballR = mission.EnvRef.Balls[0].RadiusMm;
            //log.WriteLine(mission.CommonPara.MsPerCycle);
            // log.WriteLine(ballR);
            //log.Close();
            do_fish2(mission, teamId);
            do_fish1(mission, teamId);

            return decisions;
        }
    }
}

